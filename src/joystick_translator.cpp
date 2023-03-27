
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
    
        int LOW_SERVO = 1000;
        int HIGH_SERVO = 2000;
        int MID_SERVO = 0;
        int SERVO_RANGE = 0;

        int LOW_MOTOR = 1000;
        int HIGH_MOTOR = 2000;
        int MID_MOTOR = 1500;
        int MOTOR_RANGE = 0;

    public:
        JoystickTranslator() {
        }

        void callback(const sensor_msgs::Joy::ConstPtr& msg) {
            // TODO: Stop gap solution. Figure out why the negative sign is needed
            // Get the current value of the steering axis
            float steering_axis_value = -(msg->axes[3]);

            // Get the current value of the velocity axis
            float velocity_axis_value = msg->axes[1];
            
            unsigned int steering_raw_value = (unsigned int) ((steering_axis_value * SERVO_RANGE) + MID_SERVO);
            unsigned int velocity_raw_value = (unsigned int) ((velocity_axis_value * MOTOR_RANGE) + MID_MOTOR);

            std_msgs::UInt32 steering_msg;
            steering_msg.data = steering_raw_value;
            
            std_msgs::UInt32 velocity_msg;
            velocity_msg.data = velocity_raw_value;

            // Send the message
            steering_publisher.publish(steering_msg);            
            velocity_publisher.publish(velocity_msg);    

        }

        int main(int argc, char** argv) {
            SERVO_RANGE = (HIGH_SERVO - LOW_SERVO) / 2;
            MID_SERVO = LOW_SERVO + SERVO_RANGE;

            ros::param::get("/velocity_upper", HIGH_MOTOR);
            MOTOR_RANGE = (HIGH_MOTOR - MID_MOTOR);

            ros::Subscriber subscriber; 
            subscriber = nodeHandler.subscribe("joy", 10, &JoystickTranslator::callback, this);
            steering_publisher = nodeHandler.advertise<std_msgs::UInt32>("servo_raw", 1);
            velocity_publisher = nodeHandler.advertise<std_msgs::UInt32>("velocity_raw", 1);
            

            ros::spin();
            return 1;
        }
};

int main(int argc, char** argv) {
    std::cout << "Translating joystick inputs to PWM values ..." << std::endl;

    ros::init(argc, argv, "joystick_translator");

    JoystickTranslator jt;

    return jt.main(argc, argv);
}

