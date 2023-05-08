#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
import time
import math

# equation source
# https://cjme.springeropen.com/articles/10.1007/s10033-017-0143-z

r_wheel = 0.05715 # radius of wheel (m)
m_wheel = 0.317 # mass of wheel (kg)
J = 0.5*m_wheel*(r_wheel * r_wheel)
lf = 0.1397 # distance from center of mass to front wheel (m)
lr = 0.1397 # distance from center of mass to rear wheel (m)
m2 = lf / (lf + lr) # proportion of mass over rear wheel
m_car = 2.466 # mass of car (kg)
h_center_of_gravity = 0.075 # height of center of mass (m)
m3 = (2*(m_wheel*r_wheel)+(m_car*h_center_of_gravity))/(lf+lr) # proportion of mass at center of gravity
g = 9.81 # gravity (m/s^2)

class FrictionEstimator:
	def __init__(self):
		rospy.init_node('friction_estimator')
		self.imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
		self.friction = rospy.Publisher('/friction', Float64, queue_size=1)

	def imu_callback(self,data):
		a_x = data.linear_acceleration.x
		a_y = data.linear_acceleration.y
		#F_x = m_car * a_x
		#F_y = m_car * a_y
		#F_total = (F_x ** 2 + F_y ** 2) ** 0.5
		#F_n = m_car * g
		#mu = F_total / F_n
		z = self.linear_accel_to_rad_accel(a_x, r_wheel)
		mu = (2 * J * z) / ((m2 * r_wheel * g) + (m3 * r_wheel * a_x))
		self.friction.publish(Float64(mu))

	def linear_accel_to_rad_accel(self, a, r):
		return  (a / r)		
		
if __name__ == '__main__':
	try:
		auto = FrictionEstimator()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass