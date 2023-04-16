
/* 
    This is a sample program to set the controller values. Written in C++
    because we want to potentially write publishers amd subscribers in roscpp.
    This program takes in an argument between 0.0 and 1.0, translates it into an
    integer between 1000 and 2000 microsecond value. The microsecond values are
    then converted into the quarter-microsecond system before sending it to 

*/

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

class ServoInterface {
    private:
        int file_descriptor = -1;
        std::string DEVICE_PATH = "/dev/ttyACM0";
        ros::NodeHandle nh;
        ros::Subscriber sub;

        unsigned int convertInputToMicroseconds(float input) {
            unsigned int retval = 0;
            
            // Input sanitization
            input = input < 0.0 ? 0.0: input;
            input = input > 1.0 ? 1.0: input;

            // Convert the input value into the number between 0.0 and 1.0
            input = 1000 + 1000 * input;

            // Convert the floating point value into an unsigned int value
            retval = (unsigned int) input;

            return retval;
        }

        unsigned char* getSetByteArray(unsigned char servo_num, unsigned int microseconds) {
            static unsigned char retval[] = {0x84, 0x00, 0x00, 0x00}; 

            // Set the servo number
            retval[1] = servo_num;

            // Mulitply the us by 4
            microseconds = microseconds * 4;
            
            retval[2] = microseconds & 0x7f;
            retval[3] = microseconds >> 7 & 0x7f;

            return retval;
        }

        unsigned char* getGetByteArray(unsigned char servo_num) {
            static unsigned char retval[] = {0x90, servo_num};

            return retval;
        }

        int openDevice(const std::string &path) {
            // Open the file with C style string name
            return open(path.c_str(), O_RDWR | O_NOCTTY);
        }

        int writeToDevice(int file_descriptor, unsigned char* data) {
            return write(file_descriptor, data, 4);
        }   

        unsigned char* readFromDevice(int file_descriptor, unsigned char* data) {
            static unsigned char response[] = {0x00, 0x00};
            int status = write(file_descriptor, data, sizeof(data));
           
            if (status != -1) {
                status = read(file_descriptor, response, 2);
            }

            return response;
        }

        void callback(const std_msgs::UInt32 steering_PWM) {
            ROS_DEBUG("Received steering PWM: %u", steering_PWM.data);
            
            unsigned char SERVO_NUM = 0; // TODO: Change this to a ros parameter instead
            int status;
            unsigned int microseconds;

            microseconds = steering_PWM.data;
            ROS_DEBUG("PWM in Microseconds: %u", microseconds);

            // Converting the input into quarter-microseconds
            unsigned char* setBytes = getSetByteArray(SERVO_NUM, microseconds);
            ROS_DEBUG("Setting the servo with the following bytes:");
            ROS_DEBUG("%x %x %x %x", (int)setBytes[0], (int)setBytes[1], (int)setBytes[2], (int)setBytes[3]);

            // Send your bytes to the device
            status = writeToDevice(file_descriptor, setBytes);
            if (status == -1) {
                perror("Error writing bytes to the file.");
            }
        }

    public:
        ServoInterface() {
        }

        int main(int argc, char **argv) {
            ROS_INFO("Waiting for the servo controller %s to be ready ...", DEVICE_PATH.c_str());
            // Open the device until a valid file descriptor is returned
            while (file_descriptor == -1) {
                sleep(1);
                file_descriptor = openDevice(DEVICE_PATH);
            }

            sub = nh.subscribe("/servo_raw", 10, &ServoInterface::callback, this);

            ROS_INFO("Device is ready ...");
            ROS_INFO("Waiting for new messages to come in ...");

            ros::spin();
            return 1;
        }        
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_interface");

    ServoInterface si;
    
    return si.main(argc, argv);
}

