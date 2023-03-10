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
#include "std_msgs/Float32.h"

int file_descriptor = -1;

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

void callback(const std_msgs::Float32 steering_angle) {
    std::cout << "Received steering angle: " << steering_angle.data << std::endl;
    
    float value = steering_angle.data;
    unsigned char SERVO_NUM = 0; // TODO: Change this to a ros parameter instead
    int status;
    unsigned int microseconds;

    microseconds = convertInputToMicroseconds(value);
    std::cout << "In microseconds we have: " << microseconds << std::endl;

    // Converting the input into quarter-microseconds
    unsigned char* setBytes = getSetByteArray(SERVO_NUM, microseconds);
    std::cout << "Setting the servo with the following bytes:" << std::hex;

    for (int i = 0; i < 4; i++) {
        std::cout << " " << (int)setBytes[i];
    }

    std::cout << std::endl;

    // Send your bytes to the device
    status = writeToDevice(file_descriptor, setBytes);
    if (status == -1) {
        perror("Error writing bytes to the file.");
    }

    // Read bytes from the device
    // unsigned char* readBytes = getGetByteArray(SERVO_NUM);
    // unsigned char* response = readFromDevice(file_descriptor, readBytes);

    // TODO: The higher order bits are shifted by 1. Fix this bug
    // std::cout << "Response from the device: "<< std::hex << (int)response[0]
    // << " " << (int)response[1] << std::endl;
}

int main(int argc, char** argv) {
    std::string DEVICE_PATH = "/dev/ttyACM0";
    int status;

    std::cout << "Waiting for the servo controller to be ready ..." << std::endl;

    // Open the device until a valid file descriptor is returned
    while (file_descriptor == -1) {
        sleep(1);
        file_descriptor = openDevice(DEVICE_PATH);
    }

    std::cout << "Device is ready ..." << std::endl;

    ros::init(argc, argv, "keyboard_interface");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/servo_cmds", 10, callback);
    std::cout << "Waiting for new messages to come in ...";

    ros::spin();

}

