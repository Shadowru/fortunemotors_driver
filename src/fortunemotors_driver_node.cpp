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

            if (mb == NULL) {
                throw std::runtime_error("Unable to create the libmodbus context");
            }

            //TODO: DEVICE_ID
            modbus_set_slave(mb, 2);
            if (modbus_connect(mb) == -1) {
                throw std::runtime_error("Connection failed");
                close();
            }

        }

        fortunemotors_driver::fortunemotor_msg read_motor_state(bool *error) {
            uint16_t tab_reg[32];

            startRead();

            fortunemotors_driver::fortunemotor_msg msg;

            int rc = modbus_read_registers(mb, 64, 11, tab_reg);

            if (rc == -1) {
                ROS_ERROR("NO data from MODBUS");
                *error = true;
            } else {

                int idx = 0;
                msg.V = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.I = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.Temp = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.Angle =  tab_reg[idx++] + (tab_reg[idx++] << 8) + (tab_reg[idx++]  << 16) + (tab_reg[idx++]  << 24);
                msg.Speed = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.VectAngle = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.Vectpwm = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.A = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.B = tab_reg[idx++] + (tab_reg[idx++] << 8);
                msg.C = tab_reg[idx++] + (tab_reg[idx++] << 8);
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

ros::Time current_time, last_time;

void setInstance(fortunemotors_driver_node::Fortunemotor *instance) {
    fortunemotors_instance = instance;
}

void velCallback(const geometry_msgs::Twist &vel) {
    if (fortunemotors_instance == NULL) {
        return;
    }



}

void publish_feedback(ros::Publisher odrive_pub, fortunemotors_driver::fortunemotor_msg msg) {
    odrive_pub.publish(msg);
}

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting fortunemotors_driver node");

    ros::init(argc, argv, "fortunemotors_driver");
    ros::NodeHandle node;
    ros::Rate rate(50);  // 50 hz

    std::string fortunemotors_uart;

    node.param<std::string>("fortunemotors_uart", fortunemotors_uart, "/dev/ttyS0");

    node.param<float>("base_width", base_width, 0.43);
    node.param<float>("wheel_radius", wheel_radius, 0.235 / 2);

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

            if(motor_error){

            } else {
                publish_feedback(fortunemotor_pub, feedback);
            }

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