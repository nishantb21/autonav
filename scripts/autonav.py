#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
from simple_pid import PID

max_speed = 0.55

class Autonav:
    def __init__(self):
        rospy.init_node('autonav')
        self.steering_error = rospy.Subscriber('/steering_error', Float64, self.steering_angle_callback)
        self.ir_sensor = rospy.Subscriber('/ir_raw', UInt32, self.ir_sensor_callback)
        self.stop_sign = rospy.Subscriber('/stop_sign', Bool, self.stop_sign_callback)
        self.steering_input = rospy.Publisher('servo_raw', UInt32, queue_size=10)
        self.velocity_input = rospy.Publisher('velocity_raw', UInt32, queue_size=10)
        self.steering_pid = PID(1.0, 0.1, 0.5, setpoint=0.5)
        self.steering_pid.output_limits = (0.0, 1.0)
        self.turn = False
        self.stop_sign = False

    def steering_angle_callback(self, data):
        self.control = data.data
        #self.control = self.steering_pid(float(data.data))
        #rospy.loginfo('raw data: {}'.format(data.data))
        if self.turn:
            self.steering_input.publish(UInt32(1))
            rospy.loginfo('TURN!!')
        else:
            self.steering_input.publish(UInt32(self.control))
            rospy.loginfo('steering input: {}'.format(self.control))
        
        if (self.stop_sign):
            self.velocity_input.publish(UInt32(0.5))
        else:
            self.velocity_input.publish(UInt32(0.55))

    def ir_sensor_callback(self, data):
        #rospy.loginfo('IR data: {}'.format(data.data))
        if (data.data < 200):
            self.turn = True
        else:
            self.turn = False

    def stop_sign_callback(self, data):
        if (data.data):
            self.stop_sign = True
        else:
            self.stop_sign = False

if __name__ == '__main__':
    try:
        auto = Autonav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass