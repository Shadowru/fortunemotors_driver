#include "fortunemotors_driver/fortunemotors_driver.h"

#ifdef __arm__
#define EN_485 18
#endif

namespace fortunemotors_driver_node {

    class Fortunemotor {
    public:
        Fortunemotor(std::string serial_name) : serial_name_(serial_name) {
#ifdef __arm__
            if(wiringPiSetupGpio() < 0) { //use BCM2835 Pin number table
                throw std::runtime_error("set wiringPi lib failed");
            } else {
                ROS_INFO("set wiringPi lib success !!! \r\n");
            }
            pinMode(EN_485, OUTPUT);
#endif
            startRead();

            mb = modbus_new_rtu(serial_name.c_str(), 115200, 'N', 8, 1);

            modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485);

            if (mb == NULL) {
                throw std::runtime_error("Unable to create the libmodbus context");
            }

            if (modbus_connect(mb) == -1) {
                throw std::runtime_error("Connection failed");
                close();
            }

        }

        fortunemotors_driver::fortunemotor_msg read_motor_state(bool *error) {
            uint16_t tab_reg[64];

            startRead();

            fortunemotors_driver::fortunemotor_msg msg;

            //TODO: DEVICE_ID
            modbus_set_slave(mb, 1);
            modbus_flush(mb);
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
            int rc = modbus_read_registers(mb, 64, 11, tab_reg);

            if (rc == -1) {
                ROS_ERROR("NO data from MODBUS Slave 1");
                *error = true;
            } else {
/*
                int i = 0;

                for (i=0; i < rc; i++) {
                    ROS_INFO("reg[%d]=%d (0x%X)", i, tab_reg[i], tab_reg[i]);
                }
                ROS_INFO("\n");
*/
                int idx = 0;
                msg.V1 = tab_reg[idx++];
                msg.I1 = tab_reg[idx++];
                msg.Temp1 = tab_reg[idx++];
                msg.Angle1 = tab_reg[idx++] + (tab_reg[idx++] << 16);
                msg.Speed1 = tab_reg[idx++];
                msg.VectAngle1 = tab_reg[idx++];
                msg.Vectpwm1 = tab_reg[idx++];
                msg.A1 = tab_reg[idx++];
                msg.B1 = tab_reg[idx++];
                msg.C1 = tab_reg[idx++];
            }

            modbus_set_slave(mb, 2);
            modbus_flush(mb);
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
            rc = modbus_read_registers(mb, 64, 11, tab_reg);

            if (rc == -1) {
                ROS_ERROR("NO data from MODBUS slave 2");
                *error = true;
            } else {
/*
                int i = 0;

                for (i=0; i < rc; i++) {
                    ROS_INFO("reg[%d]=%d (0x%X)", i, tab_reg[i], tab_reg[i]);
                }
                ROS_INFO("\n");
*/
                int idx = 0;
                msg.V2 = tab_reg[idx++];
                msg.I2 = tab_reg[idx++];
                msg.Temp2 = tab_reg[idx++];
                msg.Angle2 = tab_reg[idx++] + (tab_reg[idx++] << 16);
                msg.Speed2 = tab_reg[idx++];
                msg.VectAngle2 = tab_reg[idx++];
                msg.Vectpwm2 = tab_reg[idx++];
                msg.A2 = tab_reg[idx++];
                msg.B2 = tab_reg[idx++];
                msg.C2 = tab_reg[idx++];
            }

            *error = false;
            return msg;
        }


        void startRead() {
#ifdef __arm__
            digitalWrite(EN_485,LOW);
#endif
        }

        void startWrite() {
#ifdef __arm__
            digitalWrite(EN_485,HIGH);
#endif
        }

        void close() {
            modbus_close(mb);
            modbus_free(mb);
        };

    private:
        std::string serial_name_;
        modbus_t *mb;
    };

}

fortunemotors_driver_node::Fortunemotor *fortunemotors_instance = NULL;

float base_width;
float wheel_radius;
float wheel_circum;
float rpm_per_meter;

float steps_per_mm = 71.06;

ros::Time current_time, last_time;

void setInstance(fortunemotors_driver_node::Fortunemotor *instance) {
    fortunemotors_instance = instance;
}

void velCallback(const geometry_msgs::Twist &vel) {
    if (fortunemotors_instance == NULL) {
        return;
    }

    float v = vel.linear.x;
    float w = vel.angular.z;

    // m per sec
    float vr = ((2.0 * v) + (w * base_width)) / (2.0 * wheel_radius);
    float vl = ((2.0 * v) + (-1.0 * w * base_width)) / (2.0 * wheel_radius);

    float vl_speed_val = steps_per_mm * vr;

    ROS_INFO("vl_speed_val %s", vl_speed_val);

}

void publish_feedback(ros::Publisher odrive_pub, fortunemotors_driver::fortunemotor_msg msg) {
    odrive_pub.publish(msg);
}

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting fortunemotors_driver node");

    ros::init(argc, argv, "fortunemotors_driver");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 50 hz

    std::string fortunemotors_uart;

    node.param<std::string>("fortunemotors_uart", fortunemotors_uart, "/dev/ttyS0");

    node.param<float>("base_width", base_width, 0.40);
    node.param<float>("wheel_radius", wheel_radius, 0.165 / 2);

    wheel_circum = 2.0 * wheel_radius * M_PI;

    rpm_per_meter = 1 / wheel_circum;

    ROS_INFO("rpm_per_meter : %f", rpm_per_meter);

    try {
        fortunemotors_driver_node::Fortunemotor fortunemotors(fortunemotors_uart);

        setInstance(&fortunemotors);

        current_time = ros::Time::now();
        last_time = ros::Time::now();

        ros::Publisher fortunemotor_pub = node.advertise<fortunemotors_driver::fortunemotor_msg>("fortunemotor_msg",
                                                                                                 20);

        ros::Publisher fortunemotor_odometry = node.advertise<nav_msgs::Odometry>("odometry", 20);

        ros::Subscriber fortunemotor_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

        tf::TransformBroadcaster odom_broadcaster;

        bool motor_error = false;

        //TODO: replace with rate
        int counter = 0;

        while (ros::ok()) {

            fortunemotors_driver::fortunemotor_msg feedback = fortunemotors.read_motor_state(&motor_error);

            publish_feedback(fortunemotor_pub, feedback);

            ros::spinOnce();
            rate.sleep();
        }

        fortunemotors.close();
    } catch (const std::exception &err) {
        //ROS_ERROR("set wiringPi lib failed !!! \r\n");
        ROS_ERROR("Init exc : %s", err.what());
        return -1;
    }
    return 0;
}