#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class StopSignDetector():
	def __init__(self):
		rospy.init_node('stop_sign_detctor')
		self.bridge = CvBridge()

		self.color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
		self.publisher = rospy.Publisher('/stop_sign', Bool, queue_size=1)

		self.is_detected = Bool()

	def color_image_callback(self,data):
		# Load the image file
		img = self.bridge.imgmsg_to_cv2(data, "rgb8")

		# Convert the image to HSV color space
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# Define the lower and upper bounds of the red color in HSV
		lower_red = np.array([0, 70, 50])
		upper_red = np.array([10, 255, 255])

		# Create a mask for the red color in the image
		mask = cv2.inRange(hsv, lower_red, upper_red)

		# Apply morphological operations to the mask to remove noise
		kernel = np.ones((5, 5), np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

		# Detect contours in the mask
		_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Iterate over each contour and check if it is an octagon
		for contour in contours[:5]:
			approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.01, True)
			if len(approx) == 8:
				# If an octagon is found, return True
				rospy.loginfo("Detected?: {}".format(True))
				self.is_detected.data = Bool(True)
				self.publisher.publish(self.is_detected)
				break
		else:
			# If no octagon isrospy.loginfo("Error difference: {}".format(self.error)) found, return False
			rospy.loginfo("Detected?: {}".format(False))
			self.is_detected.data = Bool(False)
			self.publisher.publish(self.is_detected)

if __name__=='__main__':
	try:
		stop = StopSignDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
