#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
import time
import math
from simple_pid import PID
import sys
from select import select
import termios
import tty

max_speed = 0.55

class Autonav:
    def __init__(self):
        rospy.init_node('autonav')
        self.steering_error = rospy.Subscriber('/depth_controller_error', Float64, self.steering_angle_callback)
        self.ir_sensor = rospy.Subscriber('/ir_raw', UInt32, self.ir_sensor_callback)
        self.stop_sign = rospy.Subscriber('/stop_sign', Bool, self.stop_sign_callback)
        self.steering_input = rospy.Publisher('/servo_raw', UInt32, queue_size=10)
        self.velocity_input = rospy.Publisher('/velocity_raw', UInt32, queue_size=10)
        self.steering_pid = PID(1.3, 0.75, 0.03, setpoint=0)
        self.steering_pid.output_limits = (-500, 500)
        self.steering_pid.sample_time = 0.05
        self.last_input = 1500
        self.turn = False
        self.stop_sign = False
        self.start = False

        self.velocity_input.publish(UInt32(1500))
        self.steering_input.publish(UInt32(1500))  

        settings = self.saveTerminalSettings()
        key_timeout = rospy.get_param("~key_timeout", 0.5)

        t_end = time.time() + 60 * 0.25

        while (time.time() < t_end):
            self.steering_input.publish(UInt32(1500))
            self.velocity_input.publish(UInt32(1500))
            rospy.loginfo('waiting...')
            key = self.getKey(settings, key_timeout)
            if (key == 'a'):
                rospy.loginfo('a was pressed')
                break

        rospy.loginfo('done')
        self.start = True

    def steering_angle_callback(self, data):
        if (self.start):
            rospy.loginfo('errort: {}'.format((data.data*1000 + 1000) - 1500.0))
            rospy.loginfo('last inout: {}'.format(self.last_input))
            control = self.steering_pid((data.data*1000 + 1000) - 1500.0)
            rospy.loginfo('controller output: {}'.format(control))
            self.last_input = control
            if self.turn:
                self.steering_input.publish(UInt32(2000))
                rospy.loginfo('TURN!!')
            else:
                PWM = 1500.0 - control
                rospy.loginfo('command sent: {}'.format(PWM))
                self.steering_input.publish(UInt32(PWM))
                #rospy.loginfo('steering input: {}'.format(PWM))

            self.velocity_input.publish(UInt32(1550))
            rospy.loginfo('FORWARD!!')

            if (self.stop_sign):
                self.velocity_input.publish(UInt32(1500))
            else:
                self.velocity_input.publish(UInt32(1550))

    def ir_sensor_callback(self, data):
        if ((data.data < 200) and (data.data > 0)):
            self.turn = True
        else:
            self.turn = False
        

    def stop_sign_callback(self, data):
        if (data.data):
            self.stop_sign = True
        else:
            self.stop_sign = False

    def getKey(self, settings, timeout):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)

if __name__ == '__main__':
    try:
        auto = Autonav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass