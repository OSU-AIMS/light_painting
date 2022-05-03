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
from os.path import join, abspath, dirname

import rospy
import rospkg
import numpy as np



class imageLoader():
    """
    Load an image, calculate a few properties, make accessible via functions.
    """

    def __init__(self, filename, scale, color = True):
        """
        Load image.
        :param string filename: Filename with file extension (.tiff, .png, .jpg)
        :param float scale: Pixels height & width in meters.
        :param bool color: True if RGB color image. False if Grayscale image.
        """

        self.scale = scale

        # Absolute Filepaths
        rospack = rospkg.RosPack()
        img_path = join(rospack.get_ros_package_path('light_painting'), 'data', filename)

        # Load Image
        if color:
            self.img = cv2.imread(img_path, 1)
            cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            rospy.loginfo("Loaded COLOR image.")
        else: 
            self.img = cv2.imread(img_path, 0)
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

        coords = np.indices(self.img.shape, dtype='float') * self.scale






if __name__ == '__main__':

    canvas = imageLoader('grayscale/cloud_16x16.tif', scale=0.01, color=False)







####################################################################################################
####################################################################################################
####################################################################################################

# CWD = dirname(abspath(__file__)) # Control Working Directory - goes to script location
# RESOURCES = join(CWD,'images') # combine script location with folder name

# # Types of image resources
# BINARY = join(RESOURCES,'binary') # combine image folder location with binary
# GRAYSCALE = join(RESOURCES,'grayscale') 
# R_G_B = join(RESOURCES,'color') 

# ########### Binary Images:###################
# white_rim4x3 = 'white_rim_4x3.tif'
# white_rim3x3 = 'white_rim_3x3.tif'
# blockO = 'block-O_10x10.tif'
# aims = 'AIMS_20x5.tif'

# binary = cv2.imread(join(BINARY,white_rim3x3),0)

# ########### RGB images: #####################
# green_cross = 'green_cross.tif' # 3x3 image
# blockO_rgb = 'block_o_RGB.tif' # 10x10 image

# BGR = cv2.imread(join(R_G_B,green_cross))
# RGB = cv2.cvtColor(BGR,cv2.COLOR_BGR2RGB)

# ########### GrayScale Images ##################
# sweep_10x20 = 'sweep.tif'
# sweep_3x3 = 'sweep_3x3.tif'
# sweep_3x5 = 'sweep_3x5.tif'
# radial_9x9 = 'radial_gradient_9x9.tif'
# sweep_10x11 = 'sweep_10x11.tif'
# sweep_10x11 = 'sweep_10x11.tif'
# sweep_8x5= 'sweep_8x5.tif'
# gauss = 'gauss_1x10.tif'
# cloud = 'cloud_16x16.tif'

# GS = cv2.imread(join(GRAYSCALE,sweep_3x3),0)


