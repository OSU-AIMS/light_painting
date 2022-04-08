#!/usr/bin/env python3

import cv2
from os.path import join, abspath, dirname


CWD = dirname(abspath(__file__)) # Control Working Directory - goes to script location
RESOURCES = join(CWD,'images') # combine script location with folder name

# Types of image resources
BINARY = join(RESOURCES,'binary') # combine image folder location with binary
# print('Binary Directory:',BINARY)
GRAYSCALE = join(RESOURCES,'grayscale') 
R_G_B = join(RESOURCES,'RGB') 

#Binary Images:
white_rim4x3 = 'white_rim_4x3.tif'
blockO = 'block-O_10x10.tif'
aims = 'AIMS_20x5.tif'

#RGB images:
green_cross = 'green_cross.tif' # 3x3 image
blockO_rgb = 'block_o_RGB.tif' # 10x10 image

# binary = cv2.imread(join(BINARY,white_rim4x3),0)
# binary = cv2.imread(join(BINARY,blockO),0)
binary = cv2.imread(join(BINARY,aims),0)

BGR = cv2.imread(join(R_G_B,green_cross))
RGB = cv2.cvtColor(BGR,cv2.COLOR_BGR2RGB)

# BGR = cv2.imread(join(R_G_B,blockO_rgb))
# RGB = cv2.cvtColor(BGR,cv2.COLOR_BGR2RGB)


# print('binary',binary)
# Uncomment for debugging
# print('join(BINARY,white_rim)',join(BINARY,white_rim))
# print(binary)

