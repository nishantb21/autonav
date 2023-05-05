#!/usr/bin/env python

from __future__ import division
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.color_img_pub = rospy.Publisher('/colored_walls_debug', Image, queue_size=1)
        self.control_rate = rospy.Rate(10)  # 10 Hz
        self.walls = Walls()
        # self.camera_info_sub = rospy.Subscriber('/camera//color/camera_info', CameraInfo, )
        # self.wait_for_camera_info()
        self.fx = 615.6707153320312
        self.fy = 615.962158203125
        self.cx = 328.0010681152344
        self.cy = 241.31031799316406

    def depth_image_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.process_depth_image(depth_image)

    def process_depth_image(self, depth_image):
        # Preprocessing
        depth_image = cv2.medianBlur(depth_image, 7)
        depth_image_downsampled = depth_image[::2, ::2]

        surface_norms, Z_depth = self.calc_surface_normals(depth_image_downsampled)
        masked_walls = self.extract_surfaces(surface_norms, Z_depth)
        
        self.classify_walls(surface_norms, masked_walls)
        
        # For visual verification of method
        self.color_walls()
        
        self.follow_walls()
        
        # Publish control commands
        twist_msg = Twist()
        twist_msg.linear.x = ...  # Set linear velocity
        twist_msg.angular.z = ...  # Set angular velocity
        self.cmd_vel_pub.publish(twist_msg)

        self.control_rate.sleep()
        
    def calc_surface_normals(self, depth_image_downsampled):
        depth_image_float = depth_image_downsampled.astype(np.float32) / 1000.0  # Assuming depth values are in millimeters
        
        # compute x and y gradients
        grad_x = cv2.Sobel(depth_image_float, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_image_float, cv2.CV_32F, 0, 1, ksize=3)

        self.height, self.width = depth_image_float.shape
        v, u = np.mgrid[0:self.height, 0:self.width]
        
        # Define camera intrinsic parameters
        fx, fy = self.fx, self.fy# Focal lengths in x and y directions (in pixels)
        cx, cy = self.cx, self.cy  # Principal point coordinates (in pixels)

        Z = depth_image_float
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        # Create a 3D array to store the surface normals
        normals = np.zeros((self.height, self.width, 3), dtype=np.float32)

        # Calculate normals
        N = np.stack([-grad_x, -grad_y, -np.ones_like(Z)], axis=-1)
        valid_depth_mask = (Z > 0)
        normals[valid_depth_mask] = N[valid_depth_mask] / np.linalg.norm(N[valid_depth_mask], axis=-1, keepdims=True)
        
        return normals, Z

    def extract_surfaces(self, surface_normals, Z):
        wall_depth_threshold = 0.3 # [m] -- TODO might need to change
        potential_wall_mask = (Z > wall_depth_threshold)
        
        wall_angle_threshold = 30 # degrees -- TODO might need to change
        wall_cosine_threshold = np.cos(np.deg2rad(wall_angle_threshold))
        
        cosine_normals = surface_normals[:, :, 2]
        wall_normal_mask = (cosine_normals < wall_cosine_threshold)
        wall_mask = np.logical_and(potential_wall_mask, wall_normal_mask)
        
        # Clean up data
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        wall_mask_cleaned = cv2.morphologyEx(wall_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        wall_mask_cleaned = cv2.morphologyEx(wall_mask_cleaned, cv2.MORPH_OPEN, kernel)
        # Wall masked cleaned is a binary image with wall points marked as 1s
        return wall_mask_cleaned
    
    def classify_walls(self, normals, wall_mask):
        num_labels, labels = cv2.connectedComponents(wall_mask)
        
        for label in range(1, num_labels):
            cluster_mask = (labels == label)
            
            cluster_normals = normals[cluster_mask == 1]
            cluster_normal = np.mean(cluster_normals, axis = 0)
            
            # Dot product to find which points belong to which wall
            unit_vectors = {
                'X': np.array([1, 0, 0]),
                'Y': np.array([0, 1, 0]),
                'Z': np.array([0, 0, 1])
            }
            
            dot_products = {axis: np.dot(cluster_normal, unit_vector)
                            for axis, unit_vector in unit_vectors.items()}

            dominant_axis = max(dot_products, key=lambda x: abs(dot_products[x]))

            if dominant_axis == 'X':
                if dot_products[dominant_axis] > 0:
                    # Right wall
                    self.append_walls(self.walls.right, cluster_mask) # TODO: May need to add an axis def
                else:
                    # Left wall
                    self.append_walls(self.walls.left, cluster_mask)
            elif dominant_axis == 'Y':
                if dot_products[dominant_axis] > 0:
                    # floor
                    self.append_walls(self.walls.floor, cluster_mask)
                else:
                    # ceiling - not important
                    pass
            else:  #dominant_axis == 'Z'
                if dot_products[dominant_axis] > 0:
                    # front wall
                    self.append_walls(self.walls.front, cluster_mask)
                else:
                    # rear wall - not possible
                    pass
                
    def color_walls(self):
        color_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        colors = {
            'front_wall': (0, 0, 255), # red
            'floor': (0, 255, 0), # green
            'left_wall': (255, 0, 0), # blue
            'right_wall': (255, 0, 255), # magenta
        }
        
        color_image[self.walls.front]  == colors['front_wall']
        color_image[self.walls.floor] == colors['floor']
        color_image[self.walls.left] == colors['left_wall']
        color_image[self.walls.right] == colors['right_wall']
        
        # cv2.imshow('Color Image', color_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.color_img_pub.publish(ros_image)
        # return color_image
        
                    
    def append_walls(self, array, values):
        np.append(array, values, axis=0)
        
    # def wait_for_camera_info(self):
    #     camera_info = rospy.wait_for_message('/camera/camera_info', CameraInfo, timeout=2)
     
                
class Walls:
    def __init__(self):
        self.right = np.empty()
        self.left = np.empty()
        self.floor = np.empty()
        self.front = np.empty()

        
def main():
    rospy.init_node('wall_follower', anonymous=True)
    wall_follower = WallFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down wall follower node.")

if __name__ == '__main__':
    main()

