#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: M. Khan, A.C. Buynak
#
# Description:
# Support class to load and hold images for light_painting.


###########
# Imports #
###########

import cv2
from os.path import join

import rospy
import rospkg
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray

import numpy as np


#######################
# imageLoader() Class #
#######################

class imageLoader():
    """
    Load an image, calculate a few properties, make accessible via functions.
    """

    def __init__(self, filename, scale, color = True):
        """
        Load image and setup canvas with path planning properties.
        :param string filename: Filename with file extension (.tiff, .png, .jpg)
        :param float scale: Pixels height & width in meters.
        :param bool color: True if RGB color image. False if Grayscale image.
        """

        self.scale = scale
        self.path = PoseArray()

        # Absolute Filepaths
        rospack = rospkg.RosPack()
        img_path = join(rospack.get_path('light_painting'), 'data', filename)

        # Load Image
        if color:
            self.img = cv2.imread(img_path, 1)
            cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            self.pixelList = np.transpose(self.img.reshape(3, self.img[0].size))
            rospy.loginfo("Loaded COLOR image.")
        else: 
            self.img = cv2.imread(img_path, 0)
            self.pixelList = np.transpose(self.img.reshape(1, self.img.size))
            rospy.loginfo("Loaded GRAYSCALE image.")

        # Canvas Properties (meters)
        self.height = self.img.shape[0] * scale
        self.width = self.img.shape[1] * scale

        # Report
        rospy.loginfo("Height of Image (# rows): ", self.height)
        rospy.loginfo("Width of Image (# cols): ", self.width)


    def generatePathPlan(self):
        """
        Generate a raster path plan moving through each pixel in the image.
        """

        # Setup PoseArray
        myPath = PoseArray()
        myPath.header.frame_id = 'canvas'

        # Loop through each cell (pixel)
        coords = np.indices(self.img.shape, dtype='float')
        for r, c in np.nditer([coords[0], coords[1]]):
            
            # Build Pose
            myPose = Pose()
            myPose.position.x = r * self.scale
            myPose.position.y = c * self.scale
            myPose.position.z = 0
            myPose.orientation.x = 0
            myPose.orientation.y = 0
            myPose.orientation.z = 0
            myPose.orientation.w = 1

            # Append Pose to PoseArray list
            myPath.poses.append(myPose)
            del myPose

        # Exports
        self.path = myPath
        return self.path



###########
# Testing #
###########

if __name__ == '__main__':

    canvas = imageLoader('grayscale/cloud_16x16.tif', scale=0.01, color=False)
    path = canvas.generatePathPlan()

#EOF