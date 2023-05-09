#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConvergencePointDetector():
	def __init__(self):
		rospy.init_node('point_finder')
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
		self.error_pub = rospy.Publisher('/convergence_controller_error', Float64, queue_size=10)

	def image_callback(self, data):
		# Convert the image message to a CV2 image
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cv_image = cv_image[int(cv_image.shape[0]*0.3):int(cv_image.shape[0]*0.6), 0:cv_image.shape[1]]

		# Convert the image to grayscale
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		# Apply Canny edge detection
		edges = cv2.Canny(gray, 50, 150, apertureSize=3)
		#cv2.imshow('edges',edges)

		# Find lines using Hough transform
		lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

		# Find intersection point of the lines
		if lines is not None:
			points = []
			for i in range(len(lines)):
				for j in range(i+1, len(lines)):
					rho1, theta1 = lines[i][0]
					rho2, theta2 = lines[j][0]
					a1, b1 = np.cos(theta1), np.sin(theta1)
					a2, b2 = np.cos(theta2), np.sin(theta2)

					x0, y0 = a1 * rho1, b1 * rho1
					x1, y1 = a2 * rho2, b2 * rho2
					cv2.line(cv_image, (int(x0 + 1000*(-b1)), int(y0 + 1000*(a1))), (int(x0 - 1000*(-b1)), int(y0 - 1000*(a1))), (0, 0, 255), 2)
					cv2.line(cv_image, (int(x1 + 1000*(-b2)), int(y1 + 1000*(a2))), (int(x1 - 1000*(-b2)), int(y1 - 1000*(a2))), (0, 0, 255), 2)

					if abs(a1*b2 - a2*b1) < 1e-6:
						# Lines are parallel, skip
						continue
					
					x = int((b2*rho1 - b1*rho2) / (a1*b2 - a2*b1))
					y = int((a1*rho2 - a2*rho1) / (a1*b2 - a2*b1))

					if (x < int(cv_image.shape[1]*0.33)):
						x = int(cv_image.shape[1]*0.33)
					if (x > int(cv_image.shape[1]*0.66)):
						x = int(cv_image.shape[1]*0.66)
					if (y < 0):
						y = 0
					if (y > cv_image.shape[0]):
						y = cv_image.shape[0]
					#if (x >= 0) and (x <= cv_image.shape[1]) and (y >= 0) and (y <= cv_image.shape[0]):
					points.append((x, y))
					cv2.circle(cv_image, (x, y), 5, (255, 0, 0), -1)
	
			if len(points) > 0:
				x_avg = sum(x for x, y in points) / len(points)
				y_avg = sum(y for x, y in points) / len(points)
				cv2.line(cv_image, (x_avg, 0), (x_avg, cv_image.shape[0]), (0, 255, 0), 2)
				cv2.line(cv_image, (cv_image.shape[1]/2, 0), (cv_image.shape[1]/2, cv_image.shape[0]), (0, 0, 255), 2)
				#cv2.circle(cv_image, (x_avg, y_avg), 5, (255, 0, 0), -1)
				rospy.loginfo("Intersection point: (%d, %d)", x_avg, y_avg)

				center_of_image = (int(cv_image.shape[1] / 2), int(cv_image.shape[0] / 2))
				# Calculate the error difference in the x-direction only
				self.error = x_avg - center_of_image[0]
				# Print the error difference
				#rospy.loginfo("Error difference: {}".format(self.error))
				output_error = float((float(self.error)/float(cv_image.shape[1])))+0.5
				self.error_pub.publish(Float64(output_error))
				#rospy.loginfo("Error output: {}".format(output_error))
		# Display the image with the convergence point
		#cv2.imshow("Convergence Point Detector", cv_image)
		#cv2.waitKey(1)

	def line_intersection(self, line1, line2):
		x1, y1, x2, y2 = line1
		x3, y3, x4, y4 = line2
		determinant = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1)
		if determinant == 0:
			# Lines are parallel
			return None
		else:
			x = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / determinant
			y = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / determinant
			return (int(x), int(y))

if __name__=='__main__':
	try:
		detector = ConvergencePointDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
