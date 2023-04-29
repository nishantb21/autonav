#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
import time
import math
import sys
from select import select
import termios
import tty
import matplotlib.pyplot as plt
import atexit

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

        self.steering_pid = PIDController(0.1, 0.0, 0.0, 1500, 1000, 2000)

        self.input_data = []
        self.output_data = []
        self.time_array = []

        self.turn = False
        self.stop_sign = False
        self.start = False

        self.depth_error = 0.5
        self.convergence_error = 0.5
        self.is_ball_detected = False
        self.ball_position = None

        self.velocity_input.publish(UInt32(1500))
        self.steering_input.publish(UInt32(1500))  

        settings = self.saveTerminalSettings()
        key_timeout = rospy.get_param("~key_timeout", 0.1)

        t_end = time.time() + 60 * 5.00

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
                self.steering_pid.set_point = error
                #control = self.steering_pid.update()
		control = error*1000.0+1000.0	
                PWM = control
                rospy.loginfo('command sent: {}'.format(PWM))
                self.steering_input.publish(UInt32(PWM))
            else:
                rospy.loginfo('depth_error: {}'.format((self.depth_error*1000 + 1000)))
                rospy.loginfo('convergence_error: {}'.format((self.convergence_error*1000 + 1000)))
                #normalized_error = (self.depth_error + self.convergence_error)/2
		normalized_error = self.depth_error
                #self.steering_pid.set_point = (normalized_error*1000.0 + 1000.0)
                #control = self.steering_pid.update()
		control = normalized_error*1000.0+1000.0
                rospy.loginfo('controller output: {}'.format(control))
                self.last_input = control
                PWM = control
                rospy.loginfo('command sent: {}'.format(PWM))
                self.steering_input.publish(UInt32(PWM))
                #rospy.loginfo('steering input: {}'.format(PWM))

                self.output_data.append(PWM)
                self.input_data.append(normalized_error*1000.0 + 1000.0)
                self.time_array.append(rospy.Time.now().to_sec())
            
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
    
    def save_plot(self):
        # plot the data
        plt.plot(self.time_array, self.input_data)
        plt.plot(self.time_array, self.output_data)
        plt.xlabel('Time (s)')
        plt.ylabel('IR Data')
        plt.title('IR Raw Data')

        # save the plot as a .png file
        plt.savefig('PID.png')

class PIDController:
    def __init__(self, kp, ki, kd, set_point, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        self.out_min = out_min
        self.out_max = out_max
        
        self.last_output = 0
        self.last_set_point = set_point
        self.integral_error = 0

        self.last_time = rospy.Time.now().to_sec()
        
    def update(self):
        time_now = rospy.Time.now().to_sec()
        dt = time_now - self.last_time
        self.last_time = time_now

        # Calculate error
        error = self.set_point - self.last_output
        
        # Calculate proportional term
        proportional = self.kp * error
        
        # Calculate integral term
        self.integral_error += error * dt
        integral = self.ki * self.integral_error
        
        # Calculate derivative term
        derivative = self.kd * (self.set_point - self.last_set_point) / dt
        self.last_set_point = self.set_point
        
        # Calculate PID output
        output = proportional + integral + derivative
        output = max(min(output, self.out_max), self.out_min)  # Apply output limits
        self.last_output = output
        
        return output
         

if __name__ == '__main__':
    try:
        auto = Autonav()
        while not rospy.is_shutdown():
            auto.run()
        # register the save_plot function to be called when the node is killed
        atexit.register(auto.save_plot())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
