#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
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

straight_speed = 1540

class StopSignRun:
	def __init__(self):
		rospy.init_node('stop_sign_run')

		self.stop_sign = rospy.Subscriber('/stop_sign', Bool, self.stop_sign_callback)

		self.steering_input = rospy.Publisher('/servo_raw', UInt32, queue_size=10)
		self.velocity_input = rospy.Publisher('/velocity_raw', UInt32, queue_size=10)
		
		self.stop_sign = False
		self.start = False

		self.depth_error = 0.5
		self.convergence_error = 0.5

		self.velocity_input.publish(UInt32(1500))
		self.steering_input.publish(UInt32(1500))  

		settings = self.saveTerminalSettings()
		key_timeout = rospy.get_param("~key_timeout", 0.5)

		t_end = time.time() + 60 * 5.0

		while ((time.time() < t_end) and (not rospy.is_shutdown())):
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
			
			if (self.stop_sign):
				rospy.loginfo('STOP SIGN')
				self.velocity_input.publish(UInt32(1500))
			else:
				rospy.loginfo('FORWARD!!')
				self.velocity_input.publish(UInt32(straight_speed))
		
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
		auto = StopSignRun()
		while not rospy.is_shutdown():
			auto.run()
		# register the save_plot function to be called when the node is killed
		atexit.register(auto.save_plot())
		rospy.spin()
	except rospy.ROSInterruptException:
		pass