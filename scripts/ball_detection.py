#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BallDetector():
	def __init__(self):
		rospy.init_node('blue_ball_detector')
		self.bridge = CvBridge()

		self.color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
		self.ball_detected = rospy.Publisher('/ball_detected', Bool, queue_size=1)
		self.ball_location = rospy.Publisher('/ball_location', Bool, queue_size=1)

		self.is_detected = Bool()
		self.error = 0

	def color_image_callback(self,data):
		# Load the image file
		img = self.bridge.imgmsg_to_cv2(data, "bgr8")

		# Convert the image to HSV color space
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# Define the lower and upper bounds of the blue color in HSV
		lower_blue = np.array([90, 120, 50])
		upper_blue = np.array([130, 255, 255])

		# Create a mask for the blue color in the image
		mask = cv2.inRange(hsv, lower_blue, upper_blue)

		# Apply morphological operations to the mask to remove noise
		kernel = np.ones((5, 5), np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

		# Detect circles in the mask
		circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=2, minDist=100, param1=100, param2=30, minRadius=10, maxRadius=50)

		if circles is not None:
			# If a circle is found, return True
			#rospy.loginfo("Detected?: {}".format(True))
			self.is_detected.data = Bool(True)
			self.ball_detected.publish(self.is_detected)
			
			largest_circle = max(circles, key=cv2.contourArea)
			# Find the center of mass of the largest contour
			M = cv2.moments(largest_circle)
			self.center_of_mass = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
			# # Find the center of the image
			h, w = img.shape
			center_of_image = (int(w / 2), int(h / 2))
			# # Calculate the error difference in the x-direction only
			self.error = self.center_of_mass[0] - center_of_image[0]
			# # Print the error difference
			#rospy.loginfo("Error difference: {}".format(self.error))
			output_error = float((float(self.error)/float(w)))+0.5
			self.ball_location.publish(Float64(output_error))
			#rospy.loginfo("Error output: {}".format(output_error))
		else:
			# If no circle is found, return False
			#rospy.loginfo("Detected?: {}".format(False))
			self.is_detected = Bool(False)
			self.ball_detected.publish(self.is_detected)

if __name__=='__main__':
	try:
		stop = BallDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
