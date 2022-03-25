#!/usr/bin/env python3

from pickle import TRUE
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

'''
Summary of Arduino-Python control:
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
import py_to_ino_LIGHT_ON_OFF_test as arduino_led
import image_inputs as input_image


def main():
    rospy.init_node('light_painting', anonymous=False)
    rospy.loginfo(">> light_painting Node sucessfully created")

    binary_img = input_image.binary

    ''' Testing how to extract individual pixels from binary image
    print('binary_img[0]=',binary_img[0]) # [[255 255 255]] 1st row (0,1,2)
    print('binary_img[1]=',binary_img[1]) # [[  0   0   0]] 
    print('binary_img[2]=',binary_img[2]) # [[255 255 255]]
    ## ^ show all 9 pixels values as a row vector of 3 elements

    # BELOW: this splits into individual pixels
    # Could (use the row,column) method to access pixel values for binary image
    print('binary_img[0,0]=',binary_img[0,0]) # [255] Pixel 0 (3 items/pixel, starting index at 0)
    print('binary_img[0,1]=',binary_img[0,1]) # [255] Pixel 1 
    print('binary_img[0,2]=',binary_img[0,2]) # [255] Pixel 2

    print('binary_img[1,0]=',binary_img[1,0]) # [255] Pixel 3
    print('binary_img[1,1]=',binary_img[1,1]) # [0]   Pixel 4
    print('binary_img[1,2]=',binary_img[1,2]) # [255] Pixel 5

    print('binary_img[2,0]=',binary_img[2,0]) # [255] Pixel 6
    print('binary_img[2,1]=',binary_img[2,1]) # [255] Pixel 7
    print('binary_img[2,2]=',binary_img[2,2]) # [255] Pixel 8

    # OR Use .item() to extract values from np.ndarray FOR BINARY
    print('binary_img.item(0)',binary_img.item(0)) # [255] Pixel 0
    print('binary_img.item(1)',binary_img.item(1)) # [255] Pixel 1
    print('binary_img.item(2)',binary_img.item(2)) # [255] Pixel 2
    print('binary_img.item(3)',binary_img.item(3)) # [255] Pixel 3
    print('binary_img.item(4)',binary_img.item(4)) #   [0] Pixel 4
    print('binary_img.item(5)',binary_img.item(5)) # [255] Pixel 5
    print('binary_img.item(6)',binary_img.item(6)) # [255] Pixel 6
    print('binary_img.item(7)',binary_img.item(7)) # [255] Pixel 7
    print('binary_img.item(8)',binary_img.item(8)) # [255] Pixel 8
    '''
    
    ''' # Section of Code to test the LED & reading img & testing to see if LED turns/off at the right pixel location
    arduino_led.led_OFF()
    pixels = np.array([0,1,2,3,4,5,6,7,8])
    print('Number of pixels',pixels)
    for i in pixels:
        if binary_img.item(i) == 255:
            print('Pixel Number:=',i)
            print('Pixel Value:=',binary_img.item(i))
            print('LED ON')
            print(arduino.readline())             #read the serial data and print it as line
            # time.sleep(0.5)
            arduino_led.led_ON()
            time.sleep(1.5)
        else:
            print('Pixel Number:=',i)
            print('Pixel Value:=',binary_img.item(i))
            print('LED OFF')
            print(arduino.readline())             #read the serial data and print it as line
            # time.sleep(0.5)
            arduino_led.led_OFF()
            time.sleep(1.5)
    '''
    

    ''' 
    PSEUDOCODE FOR Robot motion to follow image
    - Robot makes first move; move = 1
    - if move = 1
        then go to binary_img[0,0] & compare each item in that pixel using 
        if binary_img.item()==255, then LED turns on
    move =+ 1
    continue on till 9 moves are made for each pixel.

    Current plan is to move robot to arbitrary location in space 
    & set that to the top left corner of the image. 
    Then incrementally move through each pixel, 
    reading the value from the image each time. 
    '''

    print("Binary: number of rows along horizontal",np.size(input_image.binary,0))
    # # Above and below print statements obtain size of image
    print("Binary: number of columns along Vertical",np.size(input_image.binary,1))

    
    '''
    # Next steps for After Spring break break: 
    How to set top left corner of image as (0,0)?
    - Plan: 
    --> set 1st movment to be top left corner of image.
    Ex: 1st movement goes to (0.3,0.5) (m)
        when robot gets to that position, read top left pixel value: If 255 = LED ON, If 0 = LED OFF
        Move sequentially through each pixel value from left to right until you reach bottom right corner
        (Or we pre-read all pixel values, get their location then send robot to ones with value of 255 so it keeps the light on-works well if 255 values are beside each other)
    
    # use if statements to read value in array
    If value = 0 (black), light off, move to next pixel
    If value = 255(white), light on 
    
    Need to figure out:
    - the spacing between the pixels.
    - 1 pixel = ?? inches (or meters)
    - Reduce robot velocity to 10% --> reduced see robot_control script (as of 3/21)
    - Fix trajectory error: "Validation failed: Trajectory doesn't start at current position."
    - Fix Trajectory error: "Validation failed: Missing velocity data for trajectory pt 0"
    '''


    # Commented out below to the end to test image reading and LED
    
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    rc.goto_all_zeros() 

    waypoints = []
    scale = 1

    wpose = rc.move_group.get_current_pose().pose

    waypoints.append(wpose)

    # TOP LEFT (30cm up and 50 cm left from all zeros)
    # Pose axis relative Robot origin axis
    wpose.position.z += 0.1
    wpose.position.y += -0.25
    
    waypoints.append(wpose)

    plan, fraction = rc.plan_cartesian_path(waypoints)
    # Added fraction because of this Github issue: 
    # https://github.com/ros-planning/moveit/issues/709

    input("Cartesian Plan: press <enter>")
    # for Python 2.7: raw_input("Cartesian Plan: press <enter>")
    rc.execute_plan(plan)


    # set 1 inch = 1 pixel?

    # Box length (m)
    MOTION_BOX_LENGTH = np.size(binary_img,0) 
    # set this to length of input image = np.size(input_image.binary,0) = 3
    MOTION_BOX_HEIGHT = np.size(binary_img,1)
     # set this to height of input image = 3

    # width is number of divisions over length
    WIDTH = MOTION_BOX_HEIGHT*MOTION_BOX_LENGTH # to get size of the image  & movements for each pixel
    # In this case, 9 pixels hopefully
    print('Width (number of movements robot will do)',WIDTH)

    # if we want to have robot stop at sides of pixel rather than middle. Width = 4?
    
    for i in range(WIDTH):
        print('Width (number of robot movements left)',WIDTH)
        wpose = rc.move_group.get_current_pose().pose
        waypoints = []
        wpose.position.y += MOTION_BOX_LENGTH/WIDTH
        wpose.position.z += MOTION_BOX_HEIGHT/WIDTH
        waypoints.append(wpose)
        rospy.loginfo(wpose)
        plan, fraction = rc.plan_cartesian_path(waypoints)
        input(f"Cartesian Plan {i}: press <enter>")

        # Turn LED ON/OFF depending upon pixel value
        if binary_img.item(i-1) == 0: 
            print('i: Robot Movement number:',i)
            print('i: Pixel index number:',i)
            print('binary_img.item(i)=',binary_img.item(i))
            print('Read Img Pixel: LED OFF')
            print(arduino.readline())             #read the serial data and print it as line
            arduino_led.led_OFF()
            time.sleep(0.5)
        else:
            print('i: Robot Movement number:',i)
            print('i: Pixel index number=',i)
            print('binary_img.item(i)',binary_img.item(i))
            print('Read Img Pixel: LED ON')
            print(arduino.readline())             #read the serial data and print it as line
            time.sleep(3)
            arduino_led.led_ON()
            time.sleep(0.5)
            # time.sleep(4)
            # arduino_led.led_OFF()

        rc.execute_plan(plan)
# Sequentially read: so how can we turn the light on & off while the robot is moving in case there is a gradient?

    input("All zeros: press <enter>")
    arduino_led.led_OFF()
    # for Python 2.7: raw_input("All zeros: press <enter>")

# Uncomment below to manually control LED through button input
    # print("Enter 1 to ON LED and 0 to OFF LED")
    # user_input = input()                  #waits until user enters data
    # arduino_led.control_led(user_input)     # call to turn LED on/off (as of 3/9: currently requires user input)
#####
    rc.goto_all_zeros()

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
    main()