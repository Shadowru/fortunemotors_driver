#include "fortunemotors_driver/fortunemotors_driver.h"

#ifdef __arm__
#define EN_485 18
#endif

#define MODE_ANGLE 1
#define MODE_SPEED 2
#define MODE_PWM 3
#define MODE_MANUAL 4
#define MODE_NONE 0

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
            mb = modbus_new_rtu(serial_name.c_str(), 115200, 'N', 8, 1);

            modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485);

            if (mb == NULL) {
                throw std::runtime_error("Unable to create the libmodbus context");
            }

            if (modbus_connect(mb) == -1) {
                throw std::runtime_error("Connection failed");
                close();
            }

            //TODO: config
            startWrite();
            set_motor_mode(1, MODE_SPEED);
            set_motor_current_limit(1, 10*1000);
            set_motor_speed_PID_P(1);
            set_motor_speed_PID_I(1);

            set_motor_mode(2, MODE_SPEED);
            set_motor_current_limit(1, 10*1000);
            set_motor_speed_PID_P(2);
            set_motor_speed_PID_I(2);
            endWrite();
        }

        void set_motor_current_limit(int device_id, int16_t current_limit){
            write_register(device_id, 10, current_limit);
        }
        void set_motor_mode(int device_id, int16_t mode){
            write_register(device_id, 0, mode);
        }
        void set_motor_speed(int device_id, int16_t speed){
            write_register(device_id, 3, speed);
        }

        void set_motor_speed_PID_P(int device_id){
            write_register(device_id, 13, 100);
        }

        void set_motor_speed_PID_I(int device_id){
            write_register(device_id, 12, 100);
        }

        void write_register(int device_id, int register_id, int16_t value){
            modbus_flush(mb);
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
            modbus_set_slave(mb, device_id);
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
            modbus_flush(mb);
            std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });

            ROS_INFO("Set register %d - value :%d for device %d", register_id, value, device_id);

            int res = modbus_write_register(mb, register_id, value);

            if(res < 0){
                ROS_ERROR("Error write value to modbus device %d", device_id);
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

                modbus_read_registers(mb, 29, 1, tab_reg);
                msg.error1 = tab_reg[0];

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
                modbus_read_registers(mb, 29, 1, tab_reg);
                msg.error2 = tab_reg[0];
            }

            *error = false;
            stopRead();

            return msg;
        }


        void startRead() {
            mtx.lock();
#ifdef __arm__
            //digitalWrite(EN_485,LOW);
#endif
        }

        void stopRead() {
            mtx.unlock();
        }

        void startWrite() {
            mtx.lock();
#ifdef __arm__
            //digitalWrite(EN_485,HIGH);
#endif
        }

        void endWrite(){
            mtx.unlock();
        }

        void close() {
            modbus_close(mb);
            modbus_free(mb);
        };

    private:
        std::string serial_name_;
        modbus_t *mb;
        std::mutex mtx;
    };

}

fortunemotors_driver_node::Fortunemotor *fortunemotors_instance = NULL;

float base_width;
float wheel_radius;
float wheel_circum;
float rpm_per_meter;

float steps_per_mm = 71.06;

ros::Time current_time, last_time;

float right_pos;
float left_pos;

ros::Time current_time, last_time;

float global_x;
float global_y;
float global_theta;

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

    float vl_speed_val = (steps_per_mm * vl) / 50;
    float vr_speed_val = (steps_per_mm * vr) / 50;

    ROS_INFO("vl_speed_val %f", vl_speed_val);
    ROS_INFO("vr_speed_val %f", vr_speed_val);

    fortunemotors_instance->startWrite();
    fortunemotors_instance->set_motor_speed(1, static_cast<int16_t>(vl_speed_val));
    fortunemotors_instance->set_motor_speed(2, static_cast<int16_t>(-1 * vr_speed_val));
    fortunemotors_instance->endWrite();

}

void publishOdometry(ros::Publisher odometry_pub, const fortunemotors_driver::fortunemotor_msg fortunemotor_msg,
                     tf::TransformBroadcaster odom_broadcaster, const ros::Time current_time,
                     const ros::Time last_time) {
    float curr_tick_right = fortunemotor_msg.Angle2;
    float curr_tick_left = fortunemotor_msg.Angle1;

    float curr_right_pos = curr_tick_right - right_pos;
    float curr_left_pos = -1.0 * (curr_tick_left - left_pos);

    right_pos = curr_tick_right;
    left_pos = curr_tick_left;

    float delta_right_wheel_in_meter = curr_right_pos / steps_per_mm;
    float delta_left_wheel_in_meter = curr_left_pos / steps_per_mm;

    float local_theta = (delta_right_wheel_in_meter - delta_left_wheel_in_meter) / base_width;

    float distance = (delta_right_wheel_in_meter + delta_left_wheel_in_meter) / 2;

    ros::Duration ros_time_elapsed = current_time - last_time;
    float time_elapsed = ros_time_elapsed.toSec();

    float local_x = cos(global_theta) * distance;
    float local_y = -sin(global_theta) * distance;

    global_x = global_x + (cos(global_theta) * local_x - sin(global_theta) * local_y);
    global_y = global_y + (sin(global_theta) * local_x + cos(global_theta) * local_y);

    global_theta += local_theta;

    //global_theta = math.atan2(math.sin(global_theta), math.cos(global_theta));

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, global_theta);

    ros::Time now_time = ros::Time::now();

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(global_x, global_y, 0.0));
    transform.setRotation(quaternion);

    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

    nav_msgs::Odometry odom;
    odom.header.stamp = now_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = global_x;
    odom.pose.pose.position.y = global_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(global_theta);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = distance / time_elapsed;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = local_theta / time_elapsed;
    odometry_pub.publish(odom);

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

        ROS_INFO("rpm_per_meter : %f", rpm_per_meter);

        current_time = ros::Time::now();
        last_time = ros::Time::now();

        ROS_INFO("Init odometry");

        fortunemotors_driver::fortunemotor_msg feedback = fortunemotors.read_motor_state(&motor_error);

        //TODO:function
        right_pos = static_cast<int16_t>(fortunemotor_msg.Angle2);
        left_pos = static_cast<int16_t>(fortunemotor_msg.Angle1);

        //TODO : read from params
        global_x = 0;
        global_y = 0;
        global_theta = 0;


        while (ros::ok()) {

            fortunemotors_driver::fortunemotor_msg feedback = fortunemotors.read_motor_state(&motor_error);

            publish_feedback(fortunemotor_pub, feedback);

            publishOdometry(odrive_odometry, feedback, odom_broadcaster, current_time, last_time);
            last_time = current_time;

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