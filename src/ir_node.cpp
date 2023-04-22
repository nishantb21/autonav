#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <algorithm>
#include <string>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

class IRNode {
    private:
        std::string DEVICE_PATH = "/dev/ttyACM0";
        int file_descriptor = -1;
        unsigned char IR_CHANNEL = 2;
        ros::NodeHandle nh;
        ros::Publisher pub; 

        unsigned char* readIR() {
            unsigned char command[] = {0x90, IR_CHANNEL};
            static unsigned char response[] = {0x00, 0x00};

            int status = write(file_descriptor, command, sizeof(command));
            if (status != -1) {
                status = read(file_descriptor, response, 2);
            }
            return response;
        }

	int openDevice(const std::string &path) {
	    return open(path.c_str(), O_RDWR | O_NOCTTY);
	}

    public:
        int main(int argc, char **argv) {
            pub = nh.advertise<std_msgs::UInt32>("ir_raw", 1);
            ros::Rate loop_rate(10);

            ROS_INFO("Waiting for the device to be ready");
            while(file_descriptor == -1) {
                sleep(1);
                file_descriptor = openDevice(DEVICE_PATH);
            }
            ROS_INFO("Device is ready. Reading positions now.");

            while(ros::ok()) {
                unsigned char *response = readIR();
		unsigned int raw_value = response[0] + 256 * response[1];

                ROS_DEBUG("Value: %d", raw_value);

		std_msgs::UInt32 msg;
		msg.data = raw_value;

		pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
            }
            return 1;
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ir_node");
    
    IRNode in;

    return in.main(argc, argv);
}
