#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import termios
import tty
import sys
from select import select

STRAIGHT = 0.4
LEFT = 0.0
RIGHT = 0.85

#from https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
def getKey(settings, timeout):

    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def autonavKeyPub():
    #for keypresses
    settings = termios.tcgetattr(sys.stdin)
    keytimeout = 0.25

    topic = "servo_cmds"
    freq = 5#frequency to publish at

    pub = rospy.Publisher(topic, Float32, queue_size=10)
    rospy.init_node('autonav_kb_pub', anonymous=True)
    rate = rospy.Rate(freq) # 10hz
    
    #initialize cmd ot straight
    cmd = STRAIGHT
    while not rospy.is_shutdown():

        # if only left arrow: send 0.0
        key = getKey(settings=settings,timeout=keytimeout)
        if key == 'a':
            cmd = LEFT


        # if only right arrow: send 0.85
        elif key == 'd':
            cmd = RIGHT

        # s key for staight is pressed
        elif key =='s': 
            cmd = STRAIGHT

        
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        autonavKeyPub()
    except rospy.ROSInterruptException:
        pass
