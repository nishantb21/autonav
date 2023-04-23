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
        self.depth_error = rospy.Subscriber('/depth_controller_error', Float64, self.depth_callback)
        self.convergence_error = rospy.Subscriber('/convergence_controller_error', Float64, self.convergence_callback)
        self.ball_detected = rospy.Subscriber('/ball_detected', Float64, self.ball_detection_callback)
        self.ball_position = rospy.Subscriber('/ball_location', Float64, self.ball_position_callback)
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

        self.depth_error = None
        self.convergence_error = None
        self.is_ball_detected = False
        self.ball_position = None

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

    def run(self):
        if (self.start):
            
            if self.turn:
                self.steering_input.publish(UInt32(1750))
                rospy.loginfo('TURN!!')
            if self.is_ball_detected:
                rospy.loginfo('AVOIDING BALL!!')
                error = self.ball_position_to_error(self.ball_position)
                control = self.steering_pid((normalized_error*1000 + 1000) - 1500.0)
                PWM = 1500.0 - control
                rospy.loginfo('command sent: {}'.format(PWM))
                self.steering_input.publish(UInt32(PWM))
            else:
                rospy.loginfo('depth_error: {}'.format((self.depth_error*1000 + 1000) - 1500.0))
                rospy.loginfo('convergence_error: {}'.format((self.convergence_error*1000 + 1000) - 1500.0))
                normalized_error = (self.depth_error + self.convergence_error)/2
                control = self.steering_pid((normalized_error*1000 + 1000) - 1500.0)
                rospy.loginfo('controller output: {}'.format(control))
                self.last_input = control
                PWM = 1500.0 - control
                rospy.loginfo('command sent: {}'.format(PWM))
                self.steering_input.publish(UInt32(PWM))
                #rospy.loginfo('steering input: {}'.format(PWM))
            
            if (self.stop_sign):
                rospy.loginfo('STOP SIGN')
                self.velocity_input.publish(UInt32(1500))
            else:
                rospy.loginfo('FORWARD!!')
                self.velocity_input.publish(UInt32(1550))

    def depth_callback(self, data):
        self.depth_error = data.data

    def convergence_callback(self, data):
        self.convergence_error = data.data

    def ball_detection_callback(self, data):
        self.is_ball_detected = data.data

    def ball_position_callback(self, data):
        self.ball_position = data.data

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
    
    def ball_position_to_error(self, position):
        if (position > 0.5):
            error = position - 0.5
        elif (position < 0.5):
            error = position + 0.5
        else:
            error = 0.0

        return error
            

if __name__ == '__main__':
    try:
        auto = Autonav()
        while(1):
            auto.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
