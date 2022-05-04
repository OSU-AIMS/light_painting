#!/usr/bin/env python3

import rospy

# ROS Data Types
from light_painting.msg import RGBState


class paintPublisher():
    """
    Convience class for assembling and publishing a RGB or Grayscale image publisher.
    Class must be initialized with an existing ROS publisher object.
    Once initialized, two methods are available for creating and publishing a message.
    """

    def __init__(self, pub_handle):
        self.pub_handle = pub_handle

    def setGrayMsg(self, v = 0):
        """
        Set each R,G,B value in msg to the same value.
        :param uint8 v: Grayscale value to equivenetly set all R,G,B values. Defaults to 0 (off).
        :return: 1
        """
        
        # Init new message object
        msg = RGBState()
        
        # Fill Message
        msg.red = int(v)
        msg.green = int(v)
        msg.blue = int(v)
        self.pub_handle.publish(msg)
        rospy.loginfo('Published Grayscale msg: Published msg: Red {},Blue {}, Green {}'.format(msg.red,msg.green,msg.blue))

        return 1


    def setColorMsg(self, r = 0, g = 0, b = 0):
        """
        Set each R,G,B value in msg to a unique value.
        :param uint8 r: Red color-value. Defaults to 0 (off).
        :param uint8 g: Green color-value. Defaults to 0 (off).
        :param uint8 b: Blue color-value. Defaults to 0 (off).
        :return: 1
        """
        
        # Init new message object & fill
        msg = RGBState()
        msg.red = int(r)
        msg.green = int(g)
        msg.blue = int(b)

        self.pub_handle.publish(msg)
        rospy.loginfo('Published RGB msg: Red {},Blue {}, Green {}'.format(msg.red,msg.green,msg.blue))

        return 1
