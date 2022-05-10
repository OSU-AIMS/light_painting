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


###########
# Imports #
###########

# ROS
import rospy
from geometry_msgs.msg import Transform, PoseArray
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
            rospy.sleep(0.5)

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

    