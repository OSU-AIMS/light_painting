#!/usr/bin/env python3

import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose

# # For Arduino Control
# import time
# import serial
# arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)

import time

# For Image Manipulation
import cv2
import numpy as np

# Custom Scripts
# import py_to_ino_LIGHT_ON_OFF_test as arduino_led
import image_inputs as input_image
from Binary_values_publisher import sendBinary2LED

# ROS Data Types
from light_painting.msg import BinaryState

#--------- Description of Script
# Script uses Serial Connection to Arduino to 
# send pixel values of Binary Image

#------------------------ Global Variables -----------------

# import image
img = input_image.binary

# Image length (m)
IMAGE_HEIGHT = np.size(img,0) 
print('height of image',IMAGE_HEIGHT) # =3
# set this to length of input image = np.size(input_image.binary,0) = 3
IMAGE_WIDTH = np.size(img,1)
    # set this to height of input image = 3
print('Width of image',IMAGE_WIDTH) #=3
PIXEL_COUNT = IMAGE_WIDTH*IMAGE_HEIGHT

row = range(IMAGE_HEIGHT) 
col = range(IMAGE_WIDTH) 
print('Row:',row)
print('Col:',col)

# Creating Real World Box Size 
MOTION_BOX_scale = 0.01 # m 
MOTION_BOX_WIDTH =  IMAGE_WIDTH*MOTION_BOX_scale # m
MOTION_BOX_HEIGHT = IMAGE_HEIGHT*MOTION_BOX_scale # m

# Starting positions for robot
z_start = 1 # m
y_start = -MOTION_BOX_WIDTH/2 # m
# ------------------------------------------------------------


def nextRow(wpose):
    # Purpose: moves robot to next row
    wpose.position.z -= MOTION_BOX_scale
    wpose.position.y = y_start # same y-axis starting value
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints

def nextColumn(wpose):
    # Purpose: increments to next column
    wpose.position.y += MOTION_BOX_scale
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints

def main():
    # input(f"To Continue press <enter>")
    move_robot = True 

    # Init Node
    pub_binary_values = rospy.Publisher('/paintbrush_binary', BinaryState, queue_size=1)
    rospy.init_node('Seurat')
    rospy.loginfo(">>seurat paint node successfully created")   
    rospy.Rate(1)

    # Initialize Robot Model
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    # If Move_robot = True, then robot will always start at all-zeros
    if move_robot:
        rc.goto_all_zeros() 

    # Based on global variables, robot goes to arbitrary start position
    # Top Left is origin (similar to image origin in computer graphics)     
    waypoints = []
    start_pose = rc.move_group.get_current_pose().pose
    start_pose.position.z = z_start 
    start_pose.position.y = y_start

    if move_robot:
        rc.goto_Pose(start_pose)
 
    for i in row:
        if i != 0:
            print('reset to next row')
            print("Pixel on row {}" .format(i))
            wpose = rc.move_group.get_current_pose().pose
            waypoints = nextRow(wpose)
       
        if move_robot:
            plan, fraction = rc.plan_cartesian_path(waypoints)
            rc.execute_plan(plan)
        
        for j in col:
            print("Pixel on row {} and col {}" .format(i,j))
            wpose = rc.move_group.get_current_pose().pose
            v = img[i,j].astype('uint8')
            print('binary value:',v)

            if j != 0:
                # if not initial column, keep moving horizontally
                print('Keep moving horizontally')
                print("Pixel on row {} and col {}" .format(i,j))
                waypoints = nextColumn(wpose)

            if move_robot:
                plan, fraction = rc.plan_cartesian_path(waypoints)
                rc.execute_plan(plan)
                sendBinary2LED(pub_binary_values,v) # sends publisher handle & binar values to Led Via ROS
                # time.sleep(0.5) # delay alters lumosity of light
                sendBinary2LED(pub_binary_values) # by default: v = 0 in sendBinary2LED function (LED is off)
                # time.sleep(0.5)
#

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




    #     # Turn LED ON/OFF depending upon pixel value
    #     if img.item(i) == 0: 
    #         arduino_led.led_OFF()
    #         time.sleep(0.5)
    #         # Delay keeps light on/off for certain amount of time for consistent lumosity
    #     else:
    #         arduino_led.led_ON() 
    #         arduino_led.led_OFF() # Turns off after every movement - remove this for continuous LED ON
    #         time.sleep(0.5)

    #     if i == 0:
    #         waypoints = []
    #         print('i=',i)
    #         print('wpose.position.y=',wpose.position.y)
    #     elif i % IMAGE_WIDTH == 0: # if i is a multiple of the image width, that means it should move to the next row
    #         print('Reached end of row, starting next row at index: ',i)
    #         wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
    #         wpose.position.y = y_start # same y-axis starting value
    #         waypoints.append(copy.deepcopy(wpose))
    #         print('i=',i)
    #         print('wpose.position.y=',wpose.position.y)

    #     else: # else keep incrementally moving horizontally across y-axis
    #             wpose.position.y += MOTION_BOX_WIDTH/IMAGE_WIDTH # Previously, MOTION_BOX_LENGTH/WIDTH = 3/9=1/3 m big jump
    #             waypoints.append(copy.deepcopy(wpose))
    #             print('i=',i)
    #             print('wpose.position.y=',wpose.position.y)
        
    #     if move_robot:
    #         plan, fraction = rc.plan_cartesian_path(waypoints)
    #         rc.execute_plan(plan)

    # rc.goto_all_zeros()