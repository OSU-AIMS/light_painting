#!/usr/bin/env python3

import rospy

# ROS Data Types
from light_painting.msg import RGBState #only need this msg


class pixel_value_publisher(object):
    '''
    Class is collection of image processing tools for obtaining pixel values from Binary, RGB, Grayscale images
    With this script, we don't need the following:
    - Binary_values publisher
    - Grayscale_values publisher
    - RGB values publisher
    '''
    def __init__(self,pub_handle):
        self.pub_handle = pub_handle
        # print('pub_handle',pub_handle)

    def Binary_or_GS_img(self,v=0):
        # Parameters: works with Binary or Grayscale
        # v= value obtained from image (0 - 255)
        
        # Init new message object
        msg = RGBState()
        # print('Binary or GrayScale img: initialized RGBState() msg object: '.msg)
        
        # Fill Message
        msg.red = v
        msg.green = v
        msg.blue = v
        self.pub_handle.publish(msg)
        print('Binary or GrayScale img: Published msg: Red {},Blue {}, Green {}'.format(msg.red,msg.green,msg.blue))


    def RGB_img(self,r=0,g=0,b=0):
        # Parameters
        # r,g,b= value obtained from image (values 0-255 range)

        # Init new message object
        msg = RGBState()
        # print('RGB img: initialized RGBState() msg object=',msg)

        
        # Fill Message
        msg.red = r
        msg.green = g
        msg.blue = b
        self.pub_handle.publish(msg)
        print('RGB img: Published msg: Red {},Blue {}, Green {}'.format(msg.red,msg.green,msg.blue))

