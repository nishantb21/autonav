#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        self.Kp = 0.01  # Proportional gain
        self.target_distance = 0.5  # Desired distance from the wall in meters
        self.oncoming_distance = 0.2  # Threshold distance for detecting oncoming wall
        self.turn_angle = 0.3  # Angle to turn when an oncoming wall is detected
        self.obstacle_detected = False  # Flag to indicate if an obstacle is detected

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg)

        # Wall following
        height, width = depth_image.shape
        roi_width = int(width * 0.1)
        left_roi = depth_image[int(height * 0.3):int(height * 0.7), :roi_width]
        right_roi = depth_image[int(height * 0.3):int(height * 0.7), -roi_width:]
        left_depth = np.nanmean(left_roi)
        right_depth = np.nanmean(right_roi)
        left_error = self.target_distance - left_depth
        right_error = self.target_distance - right_depth
        linear_speed = 0.2
        angular_speed = self.Kp * (right_error - left_error)

        # Oncoming wall detection
        front_roi = depth_image[int(height * 0.3):int(height * 0.7), int(width * 0.4):int(width * 0.6)]
        front_depth = np.nanmean(front_roi)
        if front_depth < self.oncoming_distance:
            self.obstacle_detected = True

        # If an obstacle is detected, turn
        if self.obstacle_detected:
            linear_speed = 0.0
            angular_speed = self.turn_angle

        # Publish the control commands
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        self.pub.publish(cmd_vel)

if __name__ == '__main__':
    wall_follower = WallFollower()
    rospy.spin()
