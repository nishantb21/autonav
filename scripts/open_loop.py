#!/usr/bin/python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32



def talker(PWM,duration):
    
    
    rate = rospy.Rate(10) # 10hz
    startTime = rospy.get_time()
    while (rospy.get_time() - startTime)< duration and not rospy.is_shutdown():
        pub.publish(PWM)
        rate.sleep()

if __name__ == '__main__':
    try:
        global pub
        rospy.init_node('open_loop', anonymous=True)
        pub = rospy.Publisher('velocity_raw', UInt32, queue_size=10)
        while True:
            pub.publish(1500)
            duration = 2
            PWM = input("enter PWM: ")
            talker(PWM,duration)

    except rospy.ROSInterruptException:
        pass
