#include "fortunemotors_driver/fortunemotors_driver.h"

#ifdef __arm__
#define EN_485 18
#endif

namespace fortunemotors_driver_node {

    class Fortunemotor {
    public:
        Fortunemotor(std::string serial_name) : serial_name_(serial_name) {
//            serial_ = serial_new();
//
//            if (serial_open(serial_, serial_name.c_str(), 115200) < 0) {
//                ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
//                exit(1);
//            }

            /*
            buffer_size = 50;
            serial_input_waiting(serial_, &buffer_size);
             */
#ifdef __arm__
            if(wiringPiSetupGpio() < 0) { //use BCM2835 Pin number table
                throw std::runtime_error("set wiringPi lib failed");
            } else {
                ROS_INFO("set wiringPi lib success !!! \r\n");
            }
#endif
            startRead();


            mb = modbus_new_rtu(serial_name, 115200, 'N', 8, 1);

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

        void startRead(){
#ifdef __arm__
            pinMode(EN_485, OUTPUT);
            digitalWrite(EN_485,LOW);
#endif
        }

        void startWrite(){
#ifdef __arm__
            pinMode(EN_485, OUTPUT);
            digitalWrite(EN_485,HIGH);
#endif
        }

        void close() {
//            serial_close(serial_);
            serial_free(serial_);
            modbus_free(mb);
        };

    private:
        std::string serial_name_;
        serial_t *serial_;
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
    if(fortunemotors_instance == NULL){
        return;
    }
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

        ros::Publisher fortunemotor_pub = node.advertise<fortunemotors_driver::fortunemotor_msg>("fortunemotor_msg", 20);

        ros::Publisher fortunemotor_odometry = node.advertise<nav_msgs::Odometry>("odometry", 20);

        ros::Subscriber fortunemotor_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

        tf::TransformBroadcaster odom_broadcaster;

        bool motor_error = false;

        //TODO: replace with rate
        int counter = 0;

        while (ros::ok()) {
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