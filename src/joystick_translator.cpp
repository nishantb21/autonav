
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
#include <algorithm>
#include <string>

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/Joy.h"

class JoystickTranslator {
    private:
        ros::NodeHandle nodeHandler;
        ros::Publisher steering_publisher;
        ros::Publisher velocity_publisher;

    public:
        JoystickTranslator() {
        }

        void callback(const sensor_msgs::Joy::ConstPtr& msg) {
            // Get the current value of the steering axis
            float steering_axis_value = msg->axes[3];

            // Get the current value of the velocity axis
            float velocity_axis_value = msg->axes[5];
            
            unsigned int steering_raw_value = (unsigned int) steering_axis_value;
            unsigned int velocity_raw_value = (unsigned int) velocity_axis_value;

            std_msgs::UInt32 steering_msg;
            steering_msg->data = steering_raw_value;
            
            std_msgs:UInt32 velocity_msg;
            velocity_msg->data = velocity_raw_value;

            // Send the message
            steering_publisher.publish(steering_msg);            
            velocity_publisher.publish(velocity_msg);    

        }

        int main(int argc, char** argv) {
            ros::Subscriber subscriber; 
            subscriber = nodeHandler.subscribe("joy", 10, &JoystickTranslator::callback, this);
            publisher = nodeHandler.advertise<std_msgs::UInt32>("servo_raw", 1);
            

            ros.spin();
            return 1;
        }
}

int main(int argc, char** argv) {
    std::cout << "Translating joystick inputs to PWM values ..." << std::endl;

    ros::init(argc, argv, "joystick_translator");

    JoystickTranslator jt;

    return jt.main();
}

