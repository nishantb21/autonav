#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64, UInt32, Bool
from cv_bridge import CvBridge
import cv2

class MyNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_is_corner = rospy.Publisher('/depth_turn_indicator', Bool, queue_size=10)
        self.pub_corner_debug = rospy.Publisher('/corner_detector_debug', Image, queue_size=10)
        self.sub_image = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.callback_corner)
        self.rate = rospy.Rate(10)  # 10 Hz

    def cornerDetector(self,cv_img):
        dimg = cv2.Scharr(src=cv_img, ddepth=-1, dx=9, dy=0)
        ros_img = self.bridge.cv2_to_imgmsg(dimg)
        self.pub_corner_debug.publish(ros_img)

        return False

        

    def callback_corner(self, msg):

        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        is_corner = Bool()
        is_corner.data = self.cornerDetector(cv_img)

        rospy.loginfo('Received message: %s', msg.data)

        self.pub_is_corner.publish(is_corner)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('my_node')
    node = MyNode()
    rospy.spin()
