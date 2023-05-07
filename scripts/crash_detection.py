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

		self.mag_accel_x = 0
		self.prev_accel_x = 0
		self.crash_timer = time.time()
		
	def imu_callback(self, data):
		#_,_,self.yaw = self.quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
		accel_x = data.linear_acceleration.x
		self.mag_accel_x = accel_x - self.prev_accel_x
		self.prev_accel = accel_x

		if (self.mag_accel_x < -2):
			self.crash_pub.publish(Bool(True))
			self.crash_timer = time.time()
		elif (time.time() - self.crash_timer) < 3:
			self.crash_pub.publish(Bool(True))
		else:
			self.crash_pub.publish(Bool(False))

if __name__ == '__main__':
	try:
		auto = CrashDetection()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
		