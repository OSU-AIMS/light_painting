#!/usr/bin/env python3

import numpy as np
import rospy


# Custom Script
import image_inputs as input_image
from light_painting.msg import RGBState #custom message type


#######################
## Support Functions ##
#######################

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



##########
## Main ##
##########

def main():
    rgb_img = input_image.RGB
    RGB_values(rgb_img)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    