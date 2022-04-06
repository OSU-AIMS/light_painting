#!/usr/bin/env python3

# Importing Libraries
# For Arduino Communication:
# https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

# import serial
# Refer to pyserial docs: https://pyserial.readthedocs.io/en/latest/pyserial.html
# python3 -m pip install pyserial
# import time
import numpy as np
import rospy
import sys


# ROS Data Types

# Custom Script
import image_inputs as input_image
from light_painting.msg import RGBState


# arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)
# Code from: https://www.electronicshub.org/controlling-arduino-led-python/

# def RGB(red,green,blue):

#     arduino.write(red)
#     arduino.write(green)
#     arduino.write(blue)
#     print('RGB should be on')


def RGB_values(rgb_img, i,j):       
    rospy.init_node('RGB values')
    rospy.loginfo(">>RGB Values node successfully created")

    rows = np.array([0,1,2])
    cols = np.array([0,1,2])
    print('Number of pixels',len(rows)*len(cols))

    msg = RGBState()
    pub_rgb_values = rospy.Publisher('/paintbrush_color',RGBState, queue_size=5)

    # need to recall individual pixels between moves
    # For loop below will loop through all pixels each time
    # so I added i (rows), j(columns) as parameters to get from the RGB_no_descartes script
    r,g,b = rgb_img[i,j]
    msg.red = r
    msg.green = g
    msg.blue = b
    pub_rgb_values.publish(msg)
    print('msg: \n',msg)

    IMAGE_HEIGHT = np.size(rgb_img,0) 
    IMAGE_WIDTH = np.size(rgb_img,1)
    print('lenght(height):',(range(IMAGE_HEIGHT)))

    for k in range(IMAGE_HEIGHT):
        print('value of i:',k)

    ''' Testing reading values from RGB
    # print('Size of RGB: ',rgb_img.size)
    # print('rgb_img[0,0] pixel 1 value:',rgb_img[0,0])
    # print('rgb_img.item(0) pixel 1 value:',rgb_img.item(0)) # gets only the 1st value
    # print('rgb_img[0,0] pixel 4 value(mid pixel):',rgb_img[1,1])
    # print('rgb_img[0,0] pixel 8 value(bottom right):',rgb_img[2,2])
    # red,green,blue = rgb_img[1,1]
    # print('Red value of pixel 4:',red)
    # print('Green value of Pixel 4:',green)
    # print('Blue value of pixel 4:',blue) 
    
    print('rgb_image.item(0)',rgb_img.item(0))
    print('rgb_image.item(1)',rgb_img.item(1))
    print('rgb_image.item(2)',rgb_img.item(2))
    print('rgb_image.item(3)',rgb_img.item(3))
    '''
    # for i in rows:
    #     for j in cols:
    #         r,g,b = rgb_img[i,j] # stores each slice value into r,g,b (order specific!)
    #         # Check in image_inputs.py to make sure you converted from BGR to RGB (thanks openCV)
    #         msg.red = r
    #         msg.green = g
    #         msg.blue = b
    #         pub_rgb_values.publish(msg)
    #         print('msg: \n',msg)
    #         break


def main():
    rgb_img = input_image.RGB
    row = 0
    col = 0
    RGB_values(rgb_img,row,col)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    