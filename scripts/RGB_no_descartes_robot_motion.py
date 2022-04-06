#!/usr/bin/env python3

import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose

# For Arduino Control
import time
import serial
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)

# For Image Manipulation
import cv2
import numpy as np
from PIL import Image

''' # Summary of Arduino-Python control:
- Write code in Arduino IDE (See /arduino_control/light_painting/Py_to_ino_Light_ON_OFF_test/Py_to_ino_Light_ON_OFF_test.ino)
- Write Python script to control the arduino (see py_to_ino_LIGHT_ON_OFF_test.py) 
- Once code is on Arduino, you only need to run the python script to control it. 
- Python acts as a "remote control" for the Arduino script  allows you to integrate with other python scripts easily

Benefits: 
- Can create basic Arduino code (say light turns off/on based on user input of 0/1 respectively)
- then create python functions as a control that automatically send the input of 0 or 1 to Arduino

This would allow you to run the python script normally without needing the user input to turn the LED off/on. 
If you solely used the Arduino script, you would need to have a user input to turn led off/on 
& ROS nodes to communicate between the other python scripts we have for further integration.
'''

# Custom Scripts
# import py_to_ino_RGB_LED_test as RGB_led
import image_inputs as input_image
import RGB_values_publisher as RGB_publisher

#Custom Message
from light_painting.msg import RGBState


def main():
 
    # Commented out below to the end to test image reading and LED
    
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    rc.goto_all_zeros() 

    waypoints = []
    scale = 1

    start_pose = rc.move_group.get_current_pose().pose

    # waypoints.append(wpose)

    # Pose axis relative Robot origin axis

    # MOTION_BOX_WIDTH/LENGTH is the static desired dimensions of the light painting
    # starting with small movements:
    #  10cm x 10cm box - 1 success
    # 20cm x 20cm - 1 failed, 1 ~~ success

    rgb_img = input_image.RGB


    MOTION_BOX_WIDTH =  .1
    MOTION_BOX_HEIGHT = .1
    # Starting positions for robot
    z_start = 1 # m
    y_start = -MOTION_BOX_WIDTH/2 # m

    start_pose.position.z = z_start 
    start_pose.position.y = y_start

    rc.goto_Pose(start_pose)
    # wpose.position.z += 0.01
    # wpose.position.y += -0.025
    
    # waypoints.append(wpose)

    # plan, fraction = rc.plan_cartesian_path(waypoints)
    # Added fraction because of this Github issue: 
    # https://github.com/ros-planning/moveit/issues/709

    # input("Cartesian Plan: press <enter>")
    # for Python 2.7: raw_input("Cartesian Plan: press <enter>")
    # rc.execute_plan(plan)


    # set 1 inch = 1 pixel?

    # Box length (m)
    IMAGE_HEIGHT = np.size(rgb_img,0) 
    print('height of image',IMAGE_HEIGHT) # =3
    # set this to length of input image = np.size(input_image.binary,0) = 3
    IMAGE_WIDTH = np.size(rgb_img,1)
     # set this to height of input image = 3
    print('Width of image',IMAGE_WIDTH) #=3
    PIXEL_COUNT = IMAGE_WIDTH*IMAGE_HEIGHT
  

    # if we want to have robot stop at sides of pixel rather than middle. Width = 4?

    row = range(IMAGE_HEIGHT) # [0,1,2]
    col = range(IMAGE_WIDTH) # [0,1,2]

    '''# reason for nested for loop:
    Since we're using RGB image, that has 3 values per pixel instead of 1 in binary. We need to call individual pixel values (r,g,b) per pixel location in the RGB.
    I treated the RGB as a matrix and have to call each pixel as [row,col] to get it. If I used .item() with an RGB, I'd get an indiviaul pixel value. 
    For example:
    the first pixel values for the 3x3 green_cross RGB is  red: 255 green: 0 blue: 0
    Using item.() = > rgb_image.item(0)= 255 rgb_image.item(1)=0 rgb_image.item(2)= 0
    You get each individual pixel, but since the image is a 3x3, there are 9 total pixels with 3 values in each which means the loop would have to run 27 times. If we use larger images, say 10x10, then the loop would run 300 times. 
    
    A nested for loop seems to be faster because you only need to run it the toal number of pixels (in this case of a 3x3, only 9 times)
    row = (range(IMAGE_HEIGHT))
    col = (range(IMAGE_WIDTH))
    '''
    
    for i in row: #range(PIXEL_COUNT):
        for j in col:
            # print('Width (number of robot movements left)',9-i)
            print("Pixel on row {} and col {}" .format(i,j))
            wpose = rc.move_group.get_current_pose().pose
            waypoints = []

            # waypoints.append(wpose)

            # RGB_publisher(rgb_img,i,j) 

            if j == 0:
                # waypoints.append(copy.deepcopy(wpose))
                waypoints = []
                # print('row (i)=',i)

                # print('wpose.position.y=',wpose.position.y)
            else: # else keep incrementally moving horizontally across y-axis
                wpose.position.y += MOTION_BOX_WIDTH/IMAGE_WIDTH # Previously, MOTION_BOX_LENGTH/WIDTH = 3/9=1/3 m big jump
                waypoints.append(copy.deepcopy(wpose))
                # print('row ',i)
                # print('wpose.position.y=',wpose.position.y)
            
            plan, fraction = rc.plan_cartesian_path(waypoints)
            # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically
            rc.execute_plan(plan)
            RGB_publisher.RGB_values(rgb_img,i,j,pub_rgb_values)
        if i == 0:
            print('First Row') 
        else:   
            # print('Starting row ',i)
            # 3 is starting next row of pixels. 
            #So when robot finish position 2, it should go back to starting point & move down to start position 3
            # 0 1 2: once robot reaches 2, it needs to return to 3 basically reset and go down
            # 3 4 5
            wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
            wpose.position.y = y_start # same y-axis starting value
            waypoints.append(copy.deepcopy(wpose))
            # print('row (i)=',i)
            # print('wpose.position.y=',wpose.position.y)


            
        # need to set bounds on how far it should go before starting the next row of pixels
        # wpose.position.z += MOTION_BOX_HEIGHT/WIDTH
        # waypoints.append(wpose)
        # rospy.loginfo(wpose)
        # plan, fraction = rc.plan_cartesian_path(waypoints)
        
        # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically

        
        # rc.execute_plan(plan)


        
# Currently Sequentially read: so how can we turn the light on & off while the robot is moving in case there is a gradient?
    # input("All zeros: press <enter>")    
    # # for Python 2.7: raw_input("All zeros: press <enter>")
    # rc.goto_all_zeros()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    

'''
Plan for integrating Arduino with robot: 
- Send I/O to arduino board via line of code (instead of pushbutton) Send Digital High maybe?
- This turns light on/off at certain points
- Move robot as normal in script. 
'''


'''
for ROS-Arduino Control:
Install Rosserial Library into Arduino
Rosserial Arduino Examples: https://github.com/ros-drivers/rosserial/tree/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples
'''

if __name__ == '__main__':
    try:
        pub_rgb_values = rospy.Publisher('paintbrush_color',RGBState, queue_size=5)       
        rospy.init_node('RGBvalues')
        rospy.loginfo(">>RGB Values node successfully created")
        rospy.Rate(1)

        # RGB_publisher.RGB_values(rgb_img)
        main()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    