#!/usr/bin/env python3

import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose


import time

# For Image Manipulation
import cv2
import numpy as np
from PIL import Image

# Custom Scripts
# import py_to_ino_RGB_LED_test as RGB_led
import image_inputs as input_image
from RGB_values_publisher import sendRGB2LED

#Custom Message
from light_painting.msg import RGBState


def main():

    move_robot = True 

    # init node & PUblishers
    pub_rgb_values = rospy.Publisher('paintbrush_color',RGBState, queue_size=5)       
    rospy.init_node('picasso')
    rospy.loginfo(">>picasso node successfully created")
    rospy.Rate(1)

    # Commented out below to the end to test image reading and LED
    
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    if move_robot:
        rc.goto_all_zeros() 

    waypoints = []
    scale = 1

    start_pose = rc.move_group.get_current_pose().pose

    # import image
    rgb_img = input_image.RGB

    # real world box size
    MOTION_BOX_WIDTH =  .1 # m
    MOTION_BOX_HEIGHT = .1

    # Starting positions for robot
    z_start = 1 # m
    y_start = -MOTION_BOX_WIDTH/2 # m

    start_pose.position.z = z_start 
    start_pose.position.y = y_start

    if move_robot:
        rc.goto_Pose(start_pose) # uncomment to remove robot motion
    

    # Box length (m)
    IMAGE_HEIGHT = np.size(rgb_img,0) 
    print('height of image',IMAGE_HEIGHT) # =3
    # set this to length of input image = np.size(input_image.binary,0) = 3
    IMAGE_WIDTH = np.size(rgb_img,1)
     # set this to height of input image = 3
    print('Width of image',IMAGE_WIDTH) #=3
    PIXEL_COUNT = IMAGE_WIDTH*IMAGE_HEIGHT
  

    # if we want to have robot stop at sides of pixel rather than middle. Width = 4?

    row = range(IMAGE_HEIGHT) # [0,1,2]
    col = range(IMAGE_WIDTH) # [0,1,2]

    '''# reason for nested for loop:
    Since we're using RGB image, that has 3 values per pixel instead of 1 in binary. We need to call individual pixel values (r,g,b) per pixel location in the RGB.
    I treated the RGB as a matrix and have to call each pixel as [row,col] to get it. If I used .item() with an RGB, I'd get an indiviaul pixel value. 
    For example:
    the first pixel values for the 3x3 green_cross RGB is  red: 255 green: 0 blue: 0
    Using item.() = > rgb_image.item(0)= 255 rgb_image.item(1)=0 rgb_image.item(2)= 0
    You get each individual pixel, but since the image is a 3x3, there are 9 total pixels with 3 values in each which means the loop would have to run 27 times. If we use larger images, say 10x10, then the loop would run 300 times. 
    
    A nested for loop seems to be faster because you only need to run it the toal number of pixels (in this case of a 3x3, only 9 times)
    row = (range(IMAGE_HEIGHT))
    col = (range(IMAGE_WIDTH))
    '''
    
    for i in row: #range(PIXEL_COUNT):
        for j in col:
            print("Pixel on row {} and col {}" .format(i,j))
            wpose = rc.move_group.get_current_pose().pose
            waypoints = []

            waypoints.append(wpose)


            r,g,b = rgb_img[i,j].astype('uint8')



            if j == 0 and i == 0:
                print('Initial Starting Position')
                # print('row ',i)
                # print('column',j)
                waypoints = []
                print('wpose.position.y=',wpose.position.y)

            elif j % IMAGE_WIDTH == 0:
                print('Moving to next row')
                wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
                wpose.position.y = y_start # same y-axis starting value
                waypoints.append(copy.deepcopy(wpose))
                # print('row ',i)
                # print('column',j)
                # print('wpose.position.y=',wpose.position.y)

            else: # keep incrementally moving horizontally across y-axis
                print('Keep moving horizontally')
                # print('row ',i)
                # print('column',j)
                wpose.position.y += MOTION_BOX_WIDTH/IMAGE_WIDTH # Previously, MOTION_BOX_LENGTH/WIDTH = 3/9=1/3 m big jump
                waypoints.append(copy.deepcopy(wpose))
                    
            # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically
            if move_robot:
                plan, fraction = rc.plan_cartesian_path(waypoints)
                rc.execute_plan(plan)
                sendRGB2LED(pub_rgb_values,r,g,b)
                time.sleep(0.5)
                sendRGB2LED(pub_rgb_values)
                time.sleep(0.5)


        # if i == 0: 
        #     wpose.position.y = y_start # same y-axis starting value
        #     waypoints = []
        #     print('row ',i)
        #     print('column',j)
        #     # print('wpose.position.y=',wpose.position.y)

        if i % IMAGE_WIDTH == 0: #
            print('reset to next row')
            print('row ',i)
            print('column',j)

            wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
            wpose.position.y = y_start # same y-axis starting value
            waypoints.append(copy.deepcopy(wpose))
            # print('wpose.position.y=',wpose.position.y)
        else: 
            wpose.position.y += MOTION_BOX_WIDTH/IMAGE_WIDTH 
            waypoints.append(copy.deepcopy(wpose))
            print('row ',i)
            print('column',j)
            # print('wpose.position.y=',wpose.position.y)
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    