#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32,Float64,Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

max_speed = 0.55

class Visualize:
	def __init__(self):
		rospy.init_node('point_finder')
		self.bridge = CvBridge()
		self.depth_error = rospy.Subscriber('/depth_controller_error', Float64, self.depth_callback)
		self.convergence_error = rospy.Subscriber('/convergence_controller_error', Float64, self.convergence_callback)
		self.ball_detected = rospy.Subscriber('/ball_detected', Float64, self.ball_detection_callback)
		self.ball_position = rospy.Subscriber('/ball_location', Float64, self.ball_position_callback)
		self.ir_sensor = rospy.Subscriber('/ir_raw', UInt32, self.ir_sensor_callback)
		self.stop_sign = rospy.Subscriber('/stop_sign', Bool, self.stop_sign_callback)
		self.PID_sub = rospy.Subscriber('/servo_raw', UInt32, self.PID_output_callback)

		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

		self.depth_error = None
		self.convergence_error = None
		self.is_ball_detected = False
		self.ball_position = self.turn = False
		self.PID_output = None

	def image_callback(self, data):
		color_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
		self.color_image = color_image

		# Get the dimensions of the image
		h, w, d = color_image.shape

		# Calculate scaling factor
		scaling_factor = w

		# Calculate error difference between the center of mass and center of image
		scaled_depth_error = scaling_factor * self.depth_error
		scaled_convergence_error = scaling_factor * self.convergence_error
		scaled_ball_position = scaling_factor * self.ball_position

		# Draw lines on the image
		depth_line = int(scaled_depth_error)
		convergence_line = int(scaled_convergence_error)
		ball_line = int(scaled_ball_position)
		center_of_image = (int(w / 2), int(h / 2))
		cv2.line(color_image, (depth_line, 0), (depth_line, h), (0, 0, 255), 2)
		cv2.putText(color_image, text= str('Depth'), org=(depth_line, 20),
			fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
            thickness=2, lineType=cv2.LINE_AA)
		cv2.line(color_image, (convergence_line, 0), (convergence_line, h), (0, 255, 0), 2)
		cv2.putText(color_image, text= str('Convergence'), org=(convergence_line, 40),
	    	fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
            thickness=2, lineType=cv2.LINE_AA)
		if (self.is_ball_detected):
			cv2.line(color_image, (ball_line, 0), (ball_line, h), (255, 0, 255), 2)
			cv2.putText(color_image, text= str('Ball'), org=(ball_line, 60),
				fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
				thickness=2, lineType=cv2.LINE_AA)
		cv2.line(color_image, (center_of_image[0], 0), (center_of_image[0], h), (0, 0, 0), 2)
		cv2.putText(color_image, text= str('Car Center'), org=(center_of_image[0], 10),
	    	fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
            thickness=2, lineType=cv2.LINE_AA)
		cv2.line(color_image, (int(self.PID_output*w), 0), (int(self.PID_output*w), h), (255, 255, 255), 2)
		cv2.putText(color_image, text= str('PID'), org=(int(self.PID_output*w), 150),
	    	fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
            thickness=2, lineType=cv2.LINE_AA)
		#avg = int((depth_line + convergence_line)/2)
		#cv2.line(color_image, (avg, 0), (avg, h), (255, 255, 255), 2)
		#cv2.putText(color_image, text= str('PID'), org=(avg, 150),
	    #	fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,0),
        #    thickness=2, lineType=cv2.LINE_AA)

		# Show the image
		cv2.imshow('Depth Follow Controller', color_image)
		cv2.waitKey(1)

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

	def PID_output_callback(self, data):
		PID_raw = data.data
		self.PID_output = (float(PID_raw) -1000.0) / 1000.0

if __name__ == '__main__':
	try:
		auto = Visualize()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass