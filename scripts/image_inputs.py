#!/usr/bin/env python3

import cv2
from os.path import join, abspath, dirname


CWD = dirname(abspath(__file__)) # Control Working Directory - goes to script location
RESOURCES = join(CWD,'images') # combine script location with folder name

# Types of image resources
BINARY = join(RESOURCES,'binary') # combine image folder location with binary
# print('Binary Directory:',BINARY)
GRAYSCALE = join(RESOURCES,'grayscale') 
RGB = join(RESOURCES,'RGB') 

#Binary Images:
white_rim4x3 = 'white_rim_4x3.tif'
blockO = 'block-O_10x10.tif'
aims = 'AIMS_20x5.tif'

# binary = cv2.imread(join(BINARY,white_rim4x3),0)
# binary = cv2.imread(join(BINARY,blockO),0)
binary = cv2.imread(join(BINARY,aims),0)


# print('binary',binary)
# Uncomment for debugging
# print('join(BINARY,white_rim)',join(BINARY,white_rim))
# print(binary)

