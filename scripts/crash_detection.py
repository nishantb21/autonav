#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
import time
import math

class CrashDetection:
	def __init__(self):
		rospy.init_node('crash_detection')
		self.imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
		self.crash_pub = rospy.Publisher('/crash', Bool, queue_size=1)

		self.mag_accel_x = None
		self.prev_accel_x = None
		self.crash_timer = time.time()
		
	def imu_callback(self, data):
		#_,_,self.yaw = self.quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
		accel_x = data.linear_acceleration.x
		
		if (self.prev_accel_x is None):
			self.prev_accel_x = accel_x
			self.crash_pub.publish(Bool(False))
		else:
			self.mag_accel_x = accel_x - self.prev_accel_x
			if (self.mag_accel_x < -2):
				self.crash_pub.publish(Bool(True))
				self.crash_timer = time.time()
			elif (time.time() - self.crash_timer) < 1:
				self.crash_pub.publish(Bool(True))
			else:
				self.crash_pub.publish(Bool(False))

		self.prev_accel_x = accel_x

if __name__ == '__main__':
	try:
		auto = CrashDetection()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
		
