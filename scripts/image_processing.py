#!/usr/bin/env python3


import numpy as np
import rospy
import sys


# ROS Data Types
from light_painting.msg import BinaryState
from light_painting.msg import RGBState


class Image_Process(object):
    '''
    Class is collection of image processing tools for obtaining pixel values from Binary, RGB, Grayscale images
    '''
    def __init__(self,binary,RGB,GS):
        self.binary = binary
        self.RGB = RGB
        self.GS = GS

    def Binary_values(self,pub_handle,img):
        
        # Init new message object
        msg = BinaryState()
        
        # Fill Message
        msg.binary = v
        pub_handle.publish(msg)

        return

    def RGB_value(self,pub_handle,img):
        # Init new message object
        msg = RGBState()
        
        # Fill Message
        msg.red = r
        msg.green = g
        msg.blue = b
        pub_handle.publish(msg)

        return 0


    def GS_value(self,pub_handle,img):
        # Init new message object
        msg = RGBState() # using same msg type b/c RGB requires r,g,b values (3 values) while Grayscale takes 1 value for Red, Green, Blue state (all same number)
        
        # Fill Message
        msg.red = v
        msg.green = v
        msg.blue = v
        pub_handle.publish(msg)

        return 0
