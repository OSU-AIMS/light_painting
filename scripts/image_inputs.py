#!/usr/bin/env python3

import cv2
from os.path import join, abspath, dirname


CWD = dirname(abspath(__file__)) # Control Working Directory - goes to script location
RESOURCES = join(CWD,'images') # combine script location with folder name

# Types of image resources
BINARY = join(RESOURCES,'binary') # combine image folder location with binary
GRAYSCALE = join(RESOURCES,'grayscale') 
R_G_B = join(RESOURCES,'RGB') 

########### Binary Images:###################
white_rim4x3 = 'white_rim_4x3.tif'
white_rim3x3 = 'white_rim_3x3.tif'
blockO = 'block-O_10x10.tif'
aims = 'AIMS_20x5.tif'

binary = cv2.imread(join(BINARY,white_rim3x3),0)

########### RGB images: #####################
green_cross = 'green_cross.tif' # 3x3 image
blockO_rgb = 'block_o_RGB.tif' # 10x10 image

BGR = cv2.imread(join(R_G_B,green_cross))
RGB = cv2.cvtColor(BGR,cv2.COLOR_BGR2RGB)

########### GrayScale Images ##################
sweep_10x20 = 'sweep.tif'
sweep_3x3 = 'sweep_3x3.tif'
sweep_3x5 = 'sweep_3x5.tif'
radial_9x9 = 'radial_gradient_9x9.tif'
sweep_10x11 = 'sweep_10x11.tif'
sweep_10x11 = 'sweep_10x11.tif'
sweep_8x5= 'sweep_8x5.tif'
gauss = 'gauss_1x10.tif'
cloud = 'cloud_16x16.tif'

GS = cv2.imread(join(GRAYSCALE,sweep_3x3),0)


