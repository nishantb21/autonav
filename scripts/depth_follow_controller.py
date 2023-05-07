#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt32, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthFollowController:
	def __init__(self):
		rospy.init_node('depth_follow_controller')
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)
		self.error_pub = rospy.Publisher('/depth_controller_error', Float64, queue_size=10)
		self.turn_pub = rospy.Publisher('/depth_turn_indicator', Bool, queue_size=10)
		self.largest_contour = rospy.Publisher('/largest_contour', Float64, queue_size=1)
		self.error = 0
		self.depth_image = None
		self.color_image = None
		self.center_of_mass = None
	
	def depth_image_callback(self, data):
		depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
		depth_image = depth_image[int(depth_image.shape[0]*0.33):int(depth_image.shape[0]*0.66), 0:depth_image.shape[1]]
		self.depth_image = depth_image
		#cv2.imshow('Cropped Depth Image', self.depth_image)
		#cv2.waitKey(1)
		
		# Adjust exposure
		exposure_factor = 0.02
		depth_image = cv2.convertScaleAbs(depth_image, alpha=exposure_factor, beta=0)
		
		# Apply thresholding
		ret, thresh = cv2.threshold(depth_image, 127, 255, cv2.THRESH_BINARY)
		depth_clip_upper = thresh[0:int(thresh.shape[0]*0.5), 0:thresh.shape[1]]
		depth_clip_lower = thresh[int(thresh.shape[0]*0.5):thresh.shape[0], 0:thresh.shape[1]]
		#cv2.imshow('upper', depth_clip_upper)
		#cv2.imshow('lower', depth_clip_lower)
		#cv2.waitKey(1)

		# Find the contours of the white shape
		_, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#_, contours, _ = cv2.findContours(depth_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Find the largest contour
		if contours:
			largest_contour = max(contours, key=cv2.contourArea)
			self.largest_contour.publish(Float64(cv2.contourArea(largest_contour)))
			#print("upper")
			#print(np.sum(depth_clip_upper >= 250))
			#print("lower")
			#print(np.sum(depth_clip_lower >= 250))
			if (np.sum(depth_clip_upper >= 250) > 7000):
				# Find the center of mass of the largest contour
				M = cv2.moments(largest_contour)
				try:
					self.center_of_mass = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
					# Find the center of the image
					h, w = depth_image.shape
					center_of_image = (int(w / 2), int(h / 2))
					# Calculate the error difference in the x-direction only
					self.error = (self.center_of_mass[0] - w/16) - center_of_image[0]
					# Print the error difference
					#rospy.loginfo("Error difference: {}".format(self.error))
					output_error = float((float(self.error)/float(w)))+0.5
					self.error_pub.publish(Float64(output_error))
					#rospy.loginfo("Error output: {}".format(output_error))

					self.turn_pub.publish(Bool(False))
				except:
					self.error_pub.publish(0.50)
					self.turn_pub.publish(Bool(True))

			else:
				#rospy.logwarn("No white shape found in the image")
				self.error_pub.publish(0.50)
				self.turn_pub.publish(Bool(True))

		else:
			#rospy.logwarn("No white shape found in the image")
			self.error_pub.publish(0.65)
			self.turn_pub.publish(Bool(True))
	

if __name__ == '__main__':
	try:
		dfc = DepthFollowController()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
