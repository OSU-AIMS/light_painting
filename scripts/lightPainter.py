#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: M. Khan, A.C. Buynak
#
# Description:
# 
# 
# 



"""
Script does not use Descartes. Solely ROS MoveIt planner
- Uses image Processing Class
- Robot is inclined to do extraneous motion
"""



###########
# Imports #
###########

# Utilities
import copy
import time

# Image Manipulation
import cv2
import numpy as np

# ROS
import rospy
from geometry_msgs.msg import Transform, Pose, PoseArray
from light_painting.msg import RGBState

# Motion Planners
from robotControl_SimpleMover import SimpleMoverClient
from robotControl_moveit import * 

# Support Classes
from imageLoader import imageLoader
from paintPublisher import paintPublisher



##########
## MAIN ##
##########

def main():

    # Motion Parameters
    TIME_GRAY_SCALE = 2/255 #20/255 # Arbitrary time--20 sec delay for pixel value of 25


    ######
    # ROS

    # Init Paintbrush Color Publisher
    pub_paint = rospy.Publisher('/monet/paintbrush_color', RGBState, queue_size=5)
    paintColor = paintPublisher(pub_paint)

    # Init PoseArray Publisher
    pub_path = rospy.Publisher('/monet/canvasPoints', PoseArray, latch=True, queue_size=1)

    # Init Node
    rospy.init_node('monet', anonymous=False, disable_signals=True)
    rospy.loginfo(">> Node 'monet' successfully created.")
    rospy.Rate(10)


    #############
    # Load Image

    image_scale = 0.010 # meter
    canvas = imageLoader('grayscale/cloud_16x16.tif', scale=image_scale, color=False)

    # Calculate Local Raster Path across Image
    canvas.generateLocalPathPlan()

    # Transform Canvas Origin (meters)
    canvas_origin = Transform()
    canvas_origin.translation.x = 0.5
    canvas_origin.translation.y = -canvas.width/2
    canvas_origin.translation.z = 1
    canvas_origin.rotation.x = 0
    canvas_origin.rotation.y = 0.70711
    canvas_origin.rotation.z = 0
    canvas_origin.rotation.w = 0.70711

    path_wrt_fixed = canvas.transformLocalPath(canvas_origin, 'base_link')

    # Publish Path
    pub_path.publish(path_wrt_fixed)


    ########################
    # Setup Motion Planners
    rc = moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    # smc = SimpleMoverClient()


    ##############
    # Main Runner

    # Default Start Position
    rc.goto_all_zeros() 

    # Loop through Path
    try:
        for i, cell in enumerate(path_wrt_fixed.poses):
            # Move to position
            rc.goto_Pose(cell)
            # result = smc.setNewGoal(cell, 5)

            # Set paintbrush color
            paintColor.setGrayMsg(canvas.pixelList[i])

            # Pause for light
            time.sleep(0.5)

            # Reset paintbrush
            paintColor.setGrayMsg()

    except rospy.ROSInternalException:
        pass
    except KeyboardInterrupt:
        rospy.logwarn("Light Painter program interrupted by user. Returning to all-zero position.")
        pass

    # Return to default Start Position
    rc.goto_all_zeros() 



###########
## Start ##
###########





def oldMain():

    move_robot = True           # set equal to False to not move robot
    planner = 'simplemover'     # Options: moveit or simplemover



    # initialize class
    
    
    # Based on global variables, robot goes to arbitrary start position
    # Top Left is origin (similar to image origin in computer graphics)
    waypoints = []
    start_pose = rc.move_group.get_current_pose().pose
    start_pose.position.z = z_start 
    start_pose.position.y = y_start

    if move_robot:
        # rc.goto_Pose(start_pose)

        result = smc.setNewGoal(start_pose, 4)
        rospy.loginfo("Result: %s", str(result))

    # Iterate through all pixels (row->col->px)
    for i in row:
        if i  != 0: # As long as i is not 0, then move to the next row.
            # i = 0, is the first row --> no need to move to next row
            # i = 1,2 are the next two rows
            print("Pixel on row {}" .format(i))
            wpose = rc.move_group.get_current_pose().pose

            if img.any() != None: # checks if the img is grayscale or RGB automatically & access appropriate function
                if(len(img.shape)<3): # For Grayscale & Binary Images
                    print('next row')
                    pixel_value.setGrayMsg()
                    waypoints = nextRow(wpose)
                elif len(img.shape)==3: # For RGB images
                    print('next row')                
                    pixel_value.setColorMsg()
                    waypoints = nextRow(wpose)
                else:
                    print("ERROR: cannot find image")  


            ## Motion Control
            if move_robot and planner == 'moveit':
                plan, fraction = rc.plan_cartesian_path(waypoints)
                rc.execute_plan(plan)
            
            elif move_robot and planner == 'simplemover':

                goal = Pose()

                goal.position.x = waypoints[0].position.x
                goal.position.y = waypoints[0].position.y
                goal.position.z = waypoints[0].position.z

                goal.orientation.x = waypoints[0].orientation.x
                goal.orientation.y = waypoints[0].orientation.y
                goal.orientation.z = waypoints[0].orientation.z
                goal.orientation.w = waypoints[0].orientation.w

                # Send Pose Goal
                result = smc.setNewGoal(goal, 3)
                rospy.loginfo("Result: %s", str(result))
        
        

        for j in col: # Start iterating through Rows
            # print("Pixel on row {} and col {}" .format(i,j))           

            if j != 0: 
                # if not initial column, keep moving horizontally
                print('Keep moving horizontally')
                print("Pixel on row {} and col {}" .format(i,j))
                wpose = rc.move_group.get_current_pose().pose
                waypoints = nextColumn(wpose)

                # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically
                ## Motion Control
                if move_robot and planner == 'moveit':
                    plan, fraction = rc.plan_cartesian_path(waypoints)
                    rc.execute_plan(plan)
                
                elif move_robot and planner == 'simplemover':

                    goal = Pose()

                    goal.position.x = waypoints[0].position.x
                    goal.position.y = waypoints[0].position.y
                    goal.position.z = waypoints[0].position.z

                    goal.orientation.x = waypoints[0].orientation.x
                    goal.orientation.y = waypoints[0].orientation.y
                    goal.orientation.z = waypoints[0].orientation.z
                    goal.orientation.w = waypoints[0].orientation.w

                    # Send Pose Goal
                    result = smc.setNewGoal(goal, 3)
                    rospy.loginfo("Result: %s", str(result))
                
                


            if img.any() != None: # checks if the img is grayscale or RGB automatically & access appropriate function
                if(len(img.shape)<3): # For Binary & GrayScale Images
                    v = img[i,j].astype('uint8') # for Binary & GrayScale images
                    print('pixel value:',v)

                    delay_GS =v*TIME_GRAY_SCALE
                    print('Delay(sec):',delay_GS)
                    
                    pixel_value.setGrayMsg(v)
                    time.sleep(delay_GS)

                    # Turn off RGB LED
                    print('Turn off RGB LED')
                    pixel_value.setGrayMsg()
                    time.sleep(0.05)

                elif len(img.shape)==3: # For RGB images
                    delay =0.5
                    print('Delay(sec):',delay)
                    r,g,b = img[i,j].astype('uint8')
                    print("Pixel value: Red {}, Green {}, Blue {}" .format(r,g,b))
                    pixel_value.setColorMsg(r,g,b)
                    time.sleep(delay) # Delay keeps light on/off for certain amount of time for consistent lumosity
                    
                    # Turn off RGB
                    print('Turn off RGB LED')
                    pixel_value.setColorMsg() # by default r,g,b=0 in sendRGB2LED() function, sending just pub handle, turns off RGB
                    time.sleep(0.05) 
                else:
                    print("ERROR: cannot find image")  

    if move_robot:
        if img.any() != None: 
            # checks if the img is grayscale or RGB automatically & access appropriate function
            if(len(img.shape)<3):#  For ('grayscale or binary img')
                pixel_value.setGrayMsg()
                rc.goto_all_zeros()
            elif len(img.shape)==3:#  For ('Colored(RGB)')
                pixel_value.setColorMsg() # by default r,g,b=0 in sendRGB2LED() function, sending just pub handle, turns off RGB
                rc.goto_all_zeros()
            else:
                print("ERROR: cannot find image")  

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



###################
## OLD FUNCTIONS ##
###################

def nextRow(wpose):
    # Purpose: moves robot to next row
    wpose.position.z -= canvas.scale
    wpose.position.y = y_start # same y-axis starting value
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints

def nextColumn(wpose):
    # Purpose: increments to next column
    wpose.position.y += canvas.scale
    waypoints= []
    waypoints.append(copy.deepcopy(wpose))
    return waypoints






    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Light Painter program interrupted before completion.")
        print(file=sys.stderr)
    except KeyboardInterrupt:
        rospy.logwarn("Light Painter program interrupted by user. Node shutting down.")

    # Shutdown
    rospy.signal_shutdown("")

    