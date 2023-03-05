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
    retval[3] = microseconds >> 8 & 0x7f;

    return retval;
}

unsigned char* getGetByteArray(unsigned char servo_num) {
    static unsigned char retval[] = {0x90, servo_num};

    return retval;
}

// int openDevice(const std::string &path) {
    // Open the file with C style string name
    // int fd = open(path.c_str(), O_RDWR | O_NOCTTY);

int main(int argc, char** argv) {
    int c;
    float value = 0.0f;
    unsigned char servo_num = 0;
    int option_index = 0;
    unsigned int microseconds;

    static struct option long_options[] = {
        {"value", required_argument, 0, 'v'},
        {"servo", required_argument, 0, 's'}
    }; 

    while ((c = getopt_long(argc, argv, "v:s:", long_options, &option_index)) != -1) {
        switch (c) {
            case 's': {
                servo_num = std::strtoul(optarg, 0, 10) & 0x7f;
                std::cout << "Servo value: " << std::hex << servo_num << std::endl;
                break;
            }
            case 'v': {
                value = std::stof(optarg);
                std::cout << "Value received: " << optarg << std::endl;
                break;
            }
            case '?': {
                std::cout << "Got unexpected argument. Ignoring ..." << std::endl;
                break;
            }
            default:
                std::cout << "Got unexpected case." << std::endl;

        }
    } 

    // Converting the input into microseconds 
    microseconds = convertInputToMicroseconds(value);
    std::cout << "In microseconds we have: " << microseconds << std::endl;

    // Converting the input into quarter-microseconds
    unsigned char* set_bytes = getSetByteArray(servo_num, microseconds);
    std::cout << "Setting the servo with the following bytes:";

    for (int i = 0; i < 4; i++) {
        std::cout << " " << set_bytes[i];
    }
    
    std::cout << std::endl;

    return 1;
}

