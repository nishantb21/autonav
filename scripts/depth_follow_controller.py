#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthFollowController:
    def __init__(self):
        rospy.init_node('depth_follow_controller')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        self.color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
        self.PID_sub = rospy.Subscriber('/servo_raw', UInt32, self.PID_output_callback)
        self.error_pub = rospy.Publisher('/depth_controller_error', Float64, queue_size=10)
        self.error = 0
        self.depth_image = None
        self.color_image = None
        self.center_of_mass = None
        self.PID_output = None
    
    def depth_image_callback(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        self.depth_image = depth_image
        
        # Adjust exposure
        exposure_factor = 0.02
        depth_image = cv2.convertScaleAbs(depth_image, alpha=exposure_factor, beta=0)
        
        # Apply thresholding
        ret, thresh = cv2.threshold(depth_image, 127, 255, cv2.THRESH_BINARY)
        
        # Find the contours of the white shape
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, _ = cv2.findContours(depth_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            # Find the center of mass of the largest contour
            M = cv2.moments(largest_contour)
            self.center_of_mass = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
            # Find the center of the image
            h, w = depth_image.shape
            center_of_image = (int(w / 2), int(h / 2))
            # Calculate the error difference in the x-direction only
            self.error = self.center_of_mass[0] - center_of_image[0]
            # Print the error difference
            rospy.loginfo("Error difference: {}".format(self.error))
            output_error = float((float(self.error)/float(w)))+0.5
            self.error_pub.publish(Float64(output_error))
            rospy.loginfo("Error output: {}".format(output_error))

            cv2.waitKey(1)
        else:
            rospy.logwarn("No white shape found in the image")
            rospy.Publish(Float64(0.5))
    
    def color_image_callback(self, data):
        color_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        self.color_image = color_image

        # Get the resolution of the depth image and color image
        depth_shape = self.depth_image.shape
        color_shape = color_image.shape

        # Calculate scaling factor
        scaling_factor = depth_shape[1] / color_shape[1]

        # Calculate error difference between the center of mass and center of image
        scaled_error = self.error / scaling_factor

        # Get the dimensions of the image
        h, w, d = color_image.shape

        # Draw lines on the image
        center_of_mass = (int(w / 2) + scaled_error, int(h / 2))
        center_of_image = (int(w / 2), int(h / 2))
        cv2.line(color_image, (center_of_mass[0], 0), (center_of_mass[0], h), (0, 255, 0), 2)
        cv2.line(color_image, (center_of_image[0], 0), (center_of_image[0], h), (0, 0, 255), 2)
        cv2.line(color_image, (int(self.PID_output*w), 0), (int(self.PID_output*w), h), (255, 0, 0), 2)

        # Show the image
        cv2.imshow('Depth Follow Controller', color_image)
        cv2.waitKey(1)

    def PID_output_callback(self, data):
        PID_raw = data.data
        rospy.loginfo("Servo output: {}".format(data.data))
        self.PID_output = (float(PID_raw) -1000.0) / 1000.0
        rospy.loginfo("PID output: {}".format(self.PID_output))
    

if __name__ == '__main__':
    try:
        dfc = DepthFollowController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
