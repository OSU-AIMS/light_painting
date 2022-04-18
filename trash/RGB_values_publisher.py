#!/usr/bin/env python3

def sendRGB2LED(pub_handle, r = 0, g = 0, b = 0):

    # Init new message object
    msg = RGBState()
    
    # Fill Message
    msg.red = r
    msg.green = g
    msg.blue = b
    pub_handle.publish(msg)

    return 0
    


###################
#### UNIT TEST ####

import numpy as np
import rospy
import sys

# ROS Data Types
from light_painting.msg import RGBState

def main():

    # Setup Publisher
    pub_rgb_values = rospy.Publisher('/paintbrush_color', RGBState, queue_size=5)

    # Setup ROS Node
    rospy.init_node('RGB_tester')
    rospy.loginfo(">>RGB_tester node successfully created")
   
    # Set Node Cycle Rate
    rospy.Rate(10)

    # TESTING: Send Color Light
    rospy.loginfo("Red")
    sendRGB2LED(pub_rgb_values, r=255)
    rospy.sleep(2)

    rospy.loginfo("Green")
    sendRGB2LED(pub_rgb_values, g=255)
    rospy.sleep(2)

    rospy.loginfo("Blue")
    sendRGB2LED(pub_rgb_values, b=255)
    rospy.sleep(2)


    rospy.loginfo("White")
    sendRGB2LED(pub_rgb_values, r=255,g=255,b=255)
    rospy.sleep(2)


    rospy.loginfo("Off")
    sendRGB2LED(pub_rgb_values)

    # End
    print("Shutting down")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    