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


# ROS Data Types

# Custom Script
import image_inputs as input_image
import light_painting.msg #custom message type
import RGBState


# arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)
# Code from: https://www.electronicshub.org/controlling-arduino-led-python/

def RGB(red,green,blue):

    arduino.write(red)
    arduino.write(green)
    arduino.write(blue)
    print('RGB should be on')


def RGB_values(rgb_img):    
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
    '''
    
    rospy.init_node('RGB values')
    rospy.loginfo(">>RGB Values node successfully created")

    rows = np.array([0,1,2])
    cols = np.array([0,1,2])
    print('Number of pixels',len(rows)*len(cols))

    msg = RGBState()
    pub_rgb_values = rospy.Publisher('/paintbrush_color',RGBState, queue_size=5)

    for i in rows:
        for j in cols:
            r,g,b = rgb_img[i,j] # stores each slice value into r,g,b (order specific!)
            # Check in image_inputs.py to make sure you converted from BGR to RGB (thanks openCV)
            msg.red = r
            msg.green = g
            msg.blue = b
            pub_rgb_values.publish(msg)


def main():
    rgb_img = input_image.RGB
    RGB_values(rgb_img)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


class RGB_publisher():
    def __init__(self,rgb_values):
        self.rgb_values = rgb_values
    def runner(self,data):
        try:
            rgb_values = [0,0,0]
            rgb_img = input_image.rgb


            r,g,b = RGB_values(rgb_img)
            data = (r,g,b)

            rgb_values.data = list(bytearray(data))

            self.rgb_values.publish(rgb_values)
            rospy.loginfo(rgb_values)


        except rospy.ROSInterruptException:
            exit()
        except KeyboardInterrupt:
            exit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    