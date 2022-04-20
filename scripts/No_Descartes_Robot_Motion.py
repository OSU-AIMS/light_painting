#!/usr/bin/env python3

import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose


import time

# For Image Manipulation
import cv2
import numpy as np

# Custom Scripts
import image_inputs as input_image
from pixel_value_publisher import pixel_value_publisher

#Custom Message
from light_painting.msg import RGBState
"""
Script does not use Descartes. Solely ROS MoveIt planner
- Uses image Processing Class
- Need to clean up robot motion as well
"""


#######################
## MOTION PARAMETERS ##
#######################

MOTION_BOX_scale = 0.010 # m 



#################
## INPUT IMAGE ##
#################

# image imports
img = input_image.RGB
# img = input_image.GS
# img = input_image.binary

# Box length (m)
IMAGE_HEIGHT = img.shape[0] 
print('height of image/rows: ',IMAGE_HEIGHT)
IMAGE_WIDTH = img.shape[1]
print('Width of image/cols: ',IMAGE_WIDTH)
print('Image Depth: ', img.shape[2])

row = range(IMAGE_HEIGHT) # [0,1,2]
col = range(IMAGE_WIDTH) # [0,1,2]

# real world box size
MOTION_BOX_WIDTH =  IMAGE_WIDTH*MOTION_BOX_scale # m
MOTION_BOX_HEIGHT = IMAGE_HEIGHT*MOTION_BOX_scale # m

# Variable time for Grayscale
TIME_GRAY_SCALE = 2/255 #20/255 # Arbitrary time--20 sec delay for pixel value of 25


# Starting positions for robot
z_start = 1 # m - arbitrary height to get down elbow position more often
y_start = -MOTION_BOX_WIDTH/2 # m


###############
## FUNCTIONS ##
###############

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



##########
## MAIN ##
##########

def main():

    # input(f"To Continue press <enter>")

    move_robot = True # set equal to False to not move robot


    # init node & Publishers
    pub_pixel_values = rospy.Publisher('/paintbrush_color', RGBState, queue_size=5)       
    rospy.init_node('monet')
    rospy.loginfo(">>monet node successfully created")
    rospy.Rate(10)


    # initialize class
    # pub_handle is a parameter for the class
    pixel_value = pixel_value_publisher(pub_pixel_values)
    
    # Initialize Robot Model
    rc = moveManipulator('mh5l')
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

    # Iterate through all pixels (row->col->px)
    for i in row:
        
        if i  != 0: # As long as i is not 0, then move to the next row.
            # i = 0, is the first row --> no need to move to next row
            # i = 1,2 are the next two rows
            print("Pixel on row {}" .format(i))
            wpose = rc.move_group.get_current_pose().pose

            if img.any() != None: # checks if the img is grayscale or RGB automatically & access appropriate function
                if(len(img.shape)<3):
                    # print('len(img.shape)',len(img.shape))
                    # turns off LED before moving to next row
                    print('next row')
                    print ('grayscale or binary')
                    pixel_value.Binary_or_GS_img()
                    waypoints = nextRow(wpose)
                elif len(img.shape)==3:
                    # turns off LED before moving to next row
                    print('next row')                
                    print ('Colored(RGB)')
                    pixel_value.RGB_img()
                    waypoints = nextRow(wpose)
                else:
                    print("cannot find image")  

        if move_robot:
            plan, fraction = rc.plan_cartesian_path(waypoints)
            rc.execute_plan(plan)
            
        for j in col:
            # print("Pixel on row {} and col {}" .format(i,j))           

            if j != 0: 
                # if not initial column, keep moving horizontally
                print('Keep moving horizontally')
                print("Pixel on row {} and col {}" .format(i,j))
                wpose = rc.move_group.get_current_pose().pose
                waypoints = nextColumn(wpose)

            # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically
            if move_robot:
                plan, fraction = rc.plan_cartesian_path(waypoints)
                rc.execute_plan(plan)

            if img.any() != None: # checks if the img is grayscale or RGB automatically & access appropriate function
                if(len(img.shape)<3):
                    print('Binary or GS img: Moving through columns')
                    # print('len(img.shape)',len(img.shape))
                    print ('grayscale or binary img read')
                    v = img[i,j].astype('uint8') # for Binary & GrayScale images
                    print('pixel value:',v)

                    delay_GS =v*TIME_GRAY_SCALE
                    print('Delay(sec):',delay_GS)
                    
                    pixel_value.Binary_or_GS_img(v)
                    time.sleep(delay_GS)

                    # Turn off RGB LED
                    print('Turn off RGB LED')
                    pixel_value.Binary_or_GS_img()
                    time.sleep(0.05)

                    # waypoints = nextRow(wpose)
                elif len(img.shape)==3:
                    print('RGB img: Moving through columns')
                    print ('Colored(RGB) img')
                    delay =0.05
                    print('Delay(sec):',delay)
                    r,g,b = img[i,j].astype('uint8')
                    print("Pixel value: Red {}, Green {}, Blue {}" .format(r,g,b))
                    pixel_value.RGB_img(r,g,b)
                    time.sleep(delay) # Delay keeps light on/off for certain amount of time for consistent lumosity
                    
                    # Turn off RGB
                    print('Turn off RGB LED')
                    pixel_value.RGB_img() # by default r,g,b=0 in sendRGB2LED() function, sending just pub handle, turns off RGB
                    time.sleep(0.5) 
                else:
                    print("cannot find image")  

    if move_robot:
        if img.any() != None: 
            # checks if the img is grayscale or RGB automatically & access appropriate function
            if(len(img.shape)<3):
                # print ('grayscale or binary img')
                pixel_value.Binary_or_GS_img()
                rc.goto_all_zeros()
            elif len(img.shape)==3:
                # print ('Colored(RGB)')
                pixel_value.RGB_img() # by default r,g,b=0 in sendRGB2LED() function, sending just pub handle, turns off RGB
                rc.goto_all_zeros()
            else:
                print("cannot find image")  



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

    