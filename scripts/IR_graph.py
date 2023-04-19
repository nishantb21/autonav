#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt32
import matplotlib.pyplot as plt
import atexit

# initialize global variables
ir_data = []
timestamp = []

# callback function for the subscriber
def ir_raw_callback(msg):
    global ir_data, timestamp
    ir_data.append(msg.data)
    timestamp.append(rospy.Time.now().to_sec())

def save_plot():
    # plot the data
    plt.plot(timestamp, ir_data)
    plt.xlabel('Time (s)')
    plt.ylabel('IR Data')
    plt.title('IR Raw Data')

    # save the plot as a .png file
    plt.savefig('ir_raw_data.png')

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('ir_raw_graph')

    # subscribe to the /ir_raw topic
    rospy.Subscriber('/ir_raw', UInt32, ir_raw_callback)

    # register the save_plot function to be called when the node is killed
    atexit.register(save_plot)

    # spin the node until it is killed
    rospy.spin()

