#!/usr/bin/env python3

import rospy
import os
import keyboard
from std_msgs.msg import Float32

def autonavKeyPub():

    topic = "servo_cmds"
    freq = 10 #frequency to publish at

    pub = rospy.Publisher(topic, Float32, queue_size=10)
    rospy.init_node('autonav_kb_pub', anonymous=True)
    rate = rospy.Rate(freq) # 10hz
    
    while not rospy.is_shutdown():

        # if only left arrow: send 0.0
        if keyboard.is_pressed("left") and not keyboard.is_pressed("right"):
            cmd = 0.0


        # if only right arrow: send 0.85
        elif keyboard.is_pressed("right") and not keyboard.is_pressed("left"):
            cmd = 0.85

        #neither arrow key is pressed or both are pressed
        else: 
            cmd = 0.4

        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        autonavKeyPub()
    except rospy.ROSInterruptException:
        pass