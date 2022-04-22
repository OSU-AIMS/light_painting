#!/usr/bin/env python3

def sendBinary2LED(pub_handle, v=0):

    # Init new message object
    msg = RGBState()
    
    # Fill Message
    msg.red = v
    msg.green = v
    msg.blue = v
    pub_handle.publish(msg)

    return 0
    


###################
#### UNIT TEST ####
import rospy
import sys

# ROS Data Types
from light_painting.msg import BinaryState
from light_painting.msg import RGBState


def main():

    # Setup Publisher
    pub_binary_values = rospy.Publisher('/paintbrush_binary', RGBState, queue_size=1)

    # Setup ROS Node
    rospy.init_node('Binary_tester')
    rospy.loginfo(">>Binary_tester node successfully created")
   
    # Set Node Cycle Rate
    rospy.Rate(10)

    # TESTING: Send Binary Light
    rospy.loginfo("Light On")
    v=255
    sendBinary2LED(pub_binary_values, v)
    rospy.sleep(2)

    rospy.loginfo("Light Off")
    sendBinary2LED(pub_binary_values)
    rospy.sleep(2)

    # End
    print("Shutting down")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    