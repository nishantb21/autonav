#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64, UInt32, Bool
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class MyNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_is_corner = rospy.Publisher('/depth_turn_indicator', Bool, queue_size=10)
        self.pub_corner_debug = rospy.Publisher('/corner_detector_debug', Image, queue_size=1)
        self.sub_image = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.callback_corner, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def cornerDetector(self,dimg):


        ## RESIZE AND BLUR
        dimg = cv2.resize(dimg,dsize=(160,90), interpolation=cv2.INTER_LINEAR)
        dimg = cv2.blur(dimg,(2,2))

        ## VERITCAL LINE FINDING
        # dimg = cv2.Sobel(src=cv_img, ddepth=-1, dx=1, dy=0,ksize=3)
        # dimg = cv2.Scharr(src=dimg, ddepth=-1, dx=1, dy=0)

        # kernel = np.array([[1,0,-1],
        #                    [10,0,-10],
        #                    [1,0,-1]])

        # sobel5x = cv2.getDerivKernels(0, 1, 5)
        # kernel = -1*np.outer(sobel5x[0], sobel5x[1])

        vert_kernel = np.array([[-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],
                                [-1, -1, -1, 0, 1, 1, 1],])#########jgjgjgjgjgjgjgjgjgjgjgjgjjjjjjjjjjjjjjj
        dimg = cv2.filter2D(dimg, ddepth=-1, kernel=-1*vert_kernel)
        # dimg = cv2.filter2D(dimg, ddepth=-1, kernel=vert_kernal)

        
        ##PUBLISH DEBUG
        ros_img = self.bridge.cv2_to_imgmsg(dimg)
        self.pub_corner_debug.publish(ros_img)

        return False

    def callback_corner(self, msg):
        # start_time = time.time();
        cv_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        is_corner = Bool()
        is_corner.data = self.cornerDetector(cv_img)

        # rospy.loginfo('Received message: %s', msg.data)

        self.pub_is_corner.publish(is_corner)
        


if __name__ == '__main__':
    rospy.init_node('my_node')
    node = MyNode()
    rospy.spin()
