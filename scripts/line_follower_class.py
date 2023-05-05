#!/usr/bin/env python

from __future__ import division
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class LaneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(5)
        
        self.low_threshold = 50
        self.high_threshold = 150
        self.kernel_size = 5
        self.rho = 1
        self.theta = np.pi / 180
        self.threshold = 1
        self.min_line_len = 3
        self.max_line_gap = 3
        
        self.count = 0
        self.none_count = 0

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
    
        resized_image = self.resize_image(image, new_width=320)
        self.process_color_image(resized_image)
        self.rate.sleep()
        
    def process_color_image(self, image):
        self.height, self.width = image.shape[:2]
        
        # Convert to grayscale
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        blur_img = cv2.GaussianBlur(gray_img, (self.kernel_size, self.kernel_size), 0)
        edges = cv2.Canny(blur_img, self.low_threshold, self.high_threshold)
        # sobel_binary = self.sobel_edge_detection(blur_img)
        
        # masked_edges = self.define_roi(sobel_binary)
        masked_edges = self.define_roi(edges)
        
        lines = cv2.HoughLinesP(masked_edges, self.rho, self.theta, self.threshold,
                                np.array([]), minLineLength=self.min_line_len, 
                                maxLineGap=self.max_line_gap)
        
        if lines is not None:
            (left_x1, right_x1), (left_x2, right_x2) = self.calc_line_intersects(lines)  # Unpack the returned tuple
            self.add_lines_to_image(image, (left_x1, right_x1), (left_x2, right_x2))
            self.displayImage(image)
        else:
            self.displayImage(self.last_image)
            self.none_count += 1
            print("None_count", self.none_count)
            print("percentage", self.none_count / self.count * 100)
            
        self.count += 1
        
        self.last_image = image
    
    def sobel_edge_detection(self, blur_img):
        sobel_x = cv2.Sobel(blur_img, cv2.CV_64F, 1, 0, ksize=self.kernel_size)
        sobel_y = cv2.Sobel(blur_img, cv2.CV_64F, 0, 1, ksize=self.kernel_size)
        sobel_mag = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_scaled = np.uint8(255 * sobel_mag / np.max(sobel_mag))
        
        sobel_threshold = 100
        _, sobel_binary = cv2.threshold(sobel_scaled, sobel_threshold, 255, cv2.THRESH_BINARY)
        
        return sobel_binary
        
    def define_roi(self, edges):
        mask = np.zeros_like(edges)
        vertices = np.array([[(0, self.height), (self.width // 2, self.height // 2 + self.height // 4),
                              (self.width, self.height)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(edges, mask)
    
    def calc_line_intersects(self, lines):
        left_slope, left_intercept, left_count = 0, 0, 0
        right_slope, right_intercept, right_count = 0, 0, 0
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1: continue
            
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            
            if slope < -0.5:
                left_slope += slope
                left_intercept += intercept
                left_count += 1
            elif slope > 0.5:
                right_slope += slope
                right_intercept += intercept
                right_count += 1
        
        if left_count > 0:
            left_slope /= left_count
            left_intercept /= left_count
            
        if right_count > 0:
            right_slope /= right_count
            right_intercept /= right_count
            
        y1 = self.height
        y2 = self.height // 2 + self.height // 4
        
        left_x1 = int((y1 - left_intercept) / left_slope) if left_slope != 0 else 0
        left_x2 = int((y2 - left_intercept) / left_slope) if left_slope != 0 else 0
        
        right_x1 = int((y1 - right_intercept) / right_slope) if right_slope != 0 else 0
        if right_slope != 0:
           right_x2 = int((y2 - right_intercept) / right_slope)
        else:
            right_x2 = 0
        
        return ((left_x1, right_x1), (left_x2, right_x2))
    
    def add_lines_to_image(self, image, x1, x2):
        left_x1, right_x1 = x1  # Unpack the x1 values
        left_x2, right_x2 = x2  # Unpack the x2 values
        
        y1 = self.height
        y2 = self.height // 2 #+ self.height // 4
        
        line_thickness = 5
        colors = {
            'blue': (0, 0, 255),
            'red': (255, 0, 0),
            'green': (0, 255, 0)
        }
        
        cv2.line(image, (left_x1, y1), (left_x2, y2), colors['blue'], line_thickness)
        cv2.line(image, (right_x1, y1), (right_x2, y2), colors['blue'], line_thickness)

        # Display frame with lane boundary lines
        cv2.line(image, (left_x1, y1), (left_x2, y2), colors['green'], 5)
        cv2.line(image, (right_x1, y1), (right_x2, y2), colors['green'], 5)
        
        return
    
    def resize_image(self, image, new_width=320):
        height, width = image.shape[:2]
        aspect_ratio = float(height) / float(width)
        new_height = int(aspect_ratio * new_width)
        
        return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    def displayImage(self, image):
        cv2.imshow('Lane Detection', image)
        cv2.waitKey(2)
        
def main():
    rospy.init_node('lane_detector', anonymous=True)
    ld = LaneDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
