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


#------------------------ Global Variables -----------------
MOTION_BOX_scale = 0.01 # m 
# import image
img = input_image.RGB


# Box length (m)
IMAGE_HEIGHT = np.size(img,0) 
print('height of image',IMAGE_HEIGHT)
IMAGE_WIDTH = np.size(img,1)
print('Width of image',IMAGE_WIDTH)

# real world box size
MOTION_BOX_WIDTH =  IMAGE_WIDTH*MOTION_BOX_scale # m
MOTION_BOX_HEIGHT = IMAGE_HEIGHT*MOTION_BOX_scale # m

# Starting positions for robot
z_start = 1 # m
y_start = -MOTION_BOX_WIDTH/2 # m
#----------------------------------

def nextRow(wpose):
    wpose.position.z -= MOTION_BOX_scale
    wpose.position.y = y_start # same y-axis starting value
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints

def nextColumn(wpose):
    #increments to next column
    wpose.position.y += MOTION_BOX_scale
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints


def main():

    move_robot = True # set equal to False to not move robot

    # init node & PUblishers
    pub_rgb_values = rospy.Publisher('paintbrush_color',RGBState, queue_size=5)       
    rospy.init_node('picasso')
    rospy.loginfo(">>picasso node successfully created")
    rospy.Rate(1)
    
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    if move_robot:
        rc.goto_all_zeros() 

    waypoints = []
    start_pose = rc.move_group.get_current_pose().pose
    start_pose.position.z = z_start 
    start_pose.position.y = y_start

    if move_robot:
        rc.goto_Pose(start_pose)

    row = range(IMAGE_HEIGHT) # [0,1,2]
    col = range(IMAGE_WIDTH) # [0,1,2]
    
    for i in row:
        
        if i  != 0: #
            print('reset to next row')
            print("Pixel on row {} and col {}" .format(i,j))
            wpose = rc.move_group.get_current_pose().pose
            waypoints = nextRow(wpose)

        if move_robot:
            plan, fraction = rc.plan_cartesian_path(waypoints)
            rc.execute_plan(plan)

            
        for j in col:
            print("Pixel on row {} and col {}" .format(i,j))
            wpose = rc.move_group.get_current_pose().pose
            r,g,b = img[i,j].astype('uint8')

            if j != 0:
                print('Keep moving horizontally')
                waypoints = nextColumn(wpose)

            # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically
            if move_robot:
                plan, fraction = rc.plan_cartesian_path(waypoints)
                rc.execute_plan(plan)
                sendRGB2LED(pub_rgb_values,r,g,b)
                time.sleep(0.5)
                sendRGB2LED(pub_rgb_values)
                time.sleep(0.5)        
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

    