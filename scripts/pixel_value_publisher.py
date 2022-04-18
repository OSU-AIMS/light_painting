#!/usr/bin/env python3


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
    def Binary_or_GS_img(self,pub_handle,v=0):
        # Parameters: works with Binary or Grayscale
        # publisher handle
        # v= value obtained from image (0 or 255)
        
        # Init new message object
        msg = RGBState()
        
        # Fill Message
        msg.red = v
        msg.green = v
        msg.blue = v
        pub_handle.publish(msg)

        return 0

    def RGB_img(self,pub_handle,r=0,g=0,b=0):
        # Parameters
        # publisher handle
        # r,g,b= value obtained from image (values 0-255 range)

        # Init new message object
        msg = RGBState()
        
        # Fill Message
        msg.red = r
        msg.green = g
        msg.blue = b
        pub_handle.publish(msg)

        return 0

    # def GS_img(self,pub_handle,v=0):
    #     # Init new message object
    #     msg = RGBState() 
    #     # using same msg type b/c RGB requires r,g,b values (3 values) 
    #     # while Grayscale takes 1 value for Red, Green, Blue state (all same number)
        
    #     # Fill Message
    #     msg.red = v
    #     msg.green = v
    #     msg.blue = v
    #     pub_handle.publish(msg)

    #     return 0
