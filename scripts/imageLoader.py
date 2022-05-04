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
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Transform, TransformStamped

# Transforms
import tf2_geometry_msgs

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
        self.local_path = PoseArray()

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


    def generateLocalPathPlan(self) -> PoseArray:
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
        self.local_path = myPath
        return self.local_path


    def transformLocalPath(self, transform: Transform, parentFrame: String = 'base_link') -> PoseArray:
        """
        Transform local path defined with respect to CANVAS into input PARENT FRAME.
        :param String parentFrame: Name of parent tf2 coordinate frame 
        :param Transform transform: Transform from input 'parentFrame' to Canvas Origin
        :return: PoseArray
        """
        
        tf_stamped = TransformStamped()
        tf_stamped.header.frame_id = parentFrame
        tf_stamped.child_frame_id = 'canvas'
        tf_stamped.transform = transform
        
        # Convert PoseArray into list of Poses in new transformed frame
        tPath = []
        for i, iPose in enumerate(self.local_path.poses):

            # indexed pose stamped (ips)
            ips = PoseStamped()
            ips.header.frame_id = tf_stamped.child_frame_id
            ips.header.seq = i
            ips.pose = iPose

            # transformed pose stamped (nps)
            tps = tf2_geometry_msgs.do_transform_pose(ips, tf_stamped)
            tPath.append(tps)

        # transformed pose array (tpa)
        tpa = PoseArray()
        tpa.header.frame_id = parentFrame

        # Convert list of transformed Poses back into a PoseArray (strips header off PoseStamped)
        for iPose in tPath:
            tpa.poses.append(iPose.pose)
        
        return tpa





###########
# Testing #
###########

if __name__ == '__main__':

    canvas = imageLoader('grayscale/cloud_16x16.tif', scale=0.01, color=False)
    path = canvas.generateLocalPathPlan()


    # Set Canvas Origin (meters)
    canvas_origin = Transform()
    canvas_origin.translation.x = 0.5
    canvas_origin.translation.y = -canvas.width/2
    canvas_origin.translation.z = 1
    canvas_origin.rotation.x = 0
    canvas_origin.rotation.y = 0.70711
    canvas_origin.rotation.z = 0
    canvas_origin.rotation.w = 0.70711

    path_wrt_fixed = canvas.transformLocalPath(canvas_origin, 'base_link')

    print('')

#EOF