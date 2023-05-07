#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

buffer = 20

class StopSignDetector():
	def __init__(self):
		rospy.init_node('stop_sign_detctor')
		self.bridge = CvBridge()

		self.color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
		self.publisher = rospy.Publisher('/stop_sign', Bool, queue_size=1)

		self.is_detected = False

		self.detect_buffer = np.full(buffer, False)
		self.detect_iterator = 0

	def color_image_callback(self,data):
		# Load the image file
		img = self.bridge.imgmsg_to_cv2(data, "rgb8")

		# Convert the image to HSV color space
		hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

		# Define the lower and upper bounds of the red color in HSV
		lower_red = np.array([0, 70, 50])
		upper_red = np.array([10, 255, 255])

		mask1 = cv2.inRange(hsv, lower_red, upper_red)

		lower_red = np.array([170, 70, 50])
                upper_red = np.array([180, 255, 255])

                mask2 = cv2.inRange(hsv, lower_red, upper_red)

		mask = mask1 | mask2



		# Create a mask for the red color in the image
		#mask = cv2.inRange(hsv, lower_red, upper_red)
		#mask = lower_mask + upper_mask

		# Apply morphological operations to the mask to remove noise
		#kernel = np.ones((5, 5), np.uint8)
		#mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

		#result = cv2.bitwise_and(hsv, hsv, mask=mask)
		
		_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Iterate over each contour and check if it is an octagon
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		cv2.drawContours(img, contours[0], 0, (0,255,0), 5)
		approx = cv2.approxPolyDP(contours[0], cv2.arcLength(contours[0], True) * 0.01, True)
		if len(approx) == 8:
				# If an octagon is found, return True
				#rospy.loginfo("Detected?: {}".format(True))
			is_detected = True
		else:
				# If no octagon isrospy.loginfo("Error difference: {}".format(self.error)) found, return False
				#rospy.loginfo("Detected?: {}".format(False))
			is_detected = False

		self.detect_buffer[self.detect_iterator] = is_detected
		#print(is_detected)
		self.detect_iterator += 1
		if (self.detect_iterator >= (buffer - 2)):
			self.detect_iterator = 0
			if np.count_nonzero(self.detect_buffer) > 2:
				self.publisher.publish(Bool(True))
			else:
				self.publisher.publish(Bool(False))

			self.detect_buffer = np.full(buffer, False)

		cv2.imshow('mask',mask)
                cv2.waitKey(1)


if __name__=='__main__':
	try:
		stop = StopSignDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
