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

int main(int argc, char** argv) {
    int c;
    float value = 0.0f;
    unsigned char servo_num = 0;
    int option_index = 0;
    unsigned int microseconds;

    std::string DEVICE_PATH = "/dev/ttyACM0";

    int file_descriptor, status;

    static struct option long_options[] = {
        {"value", required_argument, 0, 'v'},
        {"servo", required_argument, 0, 's'}
    }; 

    while ((c = getopt_long(argc, argv, "v:s:", long_options, &option_index)) != -1) {
        switch (c) {
            case 's': {
                servo_num = std::strtoul(optarg, 0, 10) & 0x7f;
                std::cout << "Servo value: " << (int)servo_num << std::endl;
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
    unsigned char* setBytes = getSetByteArray(servo_num, microseconds);
    std::cout << "Setting the servo with the following bytes:" << std::hex;

    for (int i = 0; i < 4; i++) {
        std::cout << " " << (int)setBytes[i];
    }
    
    std::cout << std::endl;

    // Open the device
    file_descriptor = openDevice(DEVICE_PATH);
    if (file_descriptor == -1) {
        perror("Error opening the device.");
        return 1;
    }


    // Send your bytes to the device
    status = writeToDevice(file_descriptor, setBytes);
    if (status == -1) {
        perror("Error writing bytes to the file.");
        return 1;
    }

    // Read bytes from the device
    unsigned char* readBytes = getGetByteArray(servo_num);
    unsigned char* response = readFromDevice(file_descriptor, readBytes);
    std::cout << "Response from the device: "<< std::hex << (int)response[0] 
    << " " << (int)response[1] << std::endl;
    
    return 0;
}

