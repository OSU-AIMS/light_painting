#!/usr/bin/env python3

def sendGrayScale2LED(pub_handle, v=0):

    # Init new message object
    msg = GrayScale()
    
    # Fill Message
    msg.red = v
    msg.green = v
    msg.blue = v
    pub_handle.publish(msg)

    return 0
    


###################
#### UNIT TEST ####

import numpy as np
import rospy
import sys

# ROS Data Types
from light_painting.msg import GrayScale

def main():

    # Setup Publisher
    pub_GS_values = rospy.Publisher('/paintbrush_GrayScale', GrayScale, queue_size=1)

    # Setup ROS Node
    rospy.init_node('GrayScale_tester')
    rospy.loginfo(">>GrayScale_tester node successfully created")
   
    # Set Node Cycle Rate
    rospy.Rate(10)

    # TESTING: Send Color Light
    sendGrayScale2LED(pub_GS_values, v=51)
    rospy.sleep(0.5)

    sendGrayScale2LED(pub_GS_values, v=76)
    rospy.sleep(0.5)
    
    sendGrayScale2LED(pub_GS_values, v=102)
    rospy.sleep(0.5)

    sendGrayScale2LED(pub_GS_values,v=128)
    rospy.sleep(5)

    # End
    print("Shutting down")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    