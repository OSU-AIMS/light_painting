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
import py_to_ino_RGB_LED_test as RGB_led
import image_inputs as input_image

def RGB_values(image):

    




def main():
 

    ''' Testing how to extract individual pixels from RGB image
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
    

    ''' PSEUDOCODE FOR Robot motion to follow image
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

    
    '''     # Next steps for After Spring break break: 
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
    
    # rc= moveManipulator('mh5l')
    # rc.set_vel(0.1)
    # rc.set_accel(0.1)

    # rc.goto_all_zeros() 

    # waypoints = []
    # scale = 1

    # start_pose = rc.move_group.get_current_pose().pose

    # # waypoints.append(wpose)

    # # Pose axis relative Robot origin axis

    # # MOTION_BOX_WIDTH/LENGTH is the static desired dimensions of the light painting
    # # starting with small movements:
    # #  10cm x 10cm box - 1 success
    # # 20cm x 20cm - 1 failed, 1 ~~ success


    # MOTION_BOX_WIDTH =  .1
    # MOTION_BOX_HEIGHT = .1
    # # Starting positions for robot
    # z_start = 1 # m
    # y_start = -MOTION_BOX_WIDTH/2 # m

    # start_pose.position.z = z_start 
    # start_pose.position.y = y_start

    # rc.goto_Pose(start_pose)
    # # wpose.position.z += 0.01
    # # wpose.position.y += -0.025
    
    # # waypoints.append(wpose)

    # # plan, fraction = rc.plan_cartesian_path(waypoints)
    # # Added fraction because of this Github issue: 
    # # https://github.com/ros-planning/moveit/issues/709

    # # input("Cartesian Plan: press <enter>")
    # # for Python 2.7: raw_input("Cartesian Plan: press <enter>")
    # # rc.execute_plan(plan)


    # # set 1 inch = 1 pixel?

    # # Box length (m)
    # IMAGE_HEIGHT = np.size(binary_img,0) 
    # print('height of image',IMAGE_HEIGHT) # =3
    # # set this to length of input image = np.size(input_image.binary,0) = 3
    # IMAGE_WIDTH = np.size(binary_img,1)
    #  # set this to height of input image = 3
    # print('Width of image',IMAGE_WIDTH) #=3
    # PIXEL_COUNT = IMAGE_WIDTH*IMAGE_HEIGHT
  

    # # if we want to have robot stop at sides of pixel rather than middle. Width = 4?
    
    # for i in range(PIXEL_COUNT):
    #     # print('Width (number of robot movements left)',9-i)
    #     wpose = rc.move_group.get_current_pose().pose
    #     waypoints = []

    #     # waypoints.append(wpose)
        
    #     # Turn RGB LED ON/OFF depending upon pixel value
    #     # if binary_img.item(i) == 0: 
    #     #     print('i: Robot Movement number:',i)
    #     #     print('i: Pixel index number:',i)
    #     #     print('binary_img.item(i)=',binary_img.item(i))
    #     #     # print(arduino.readline())             #read the serial data and print it as line
    #     #     arduino_led.led_OFF()
    #     #     time.sleep(0.5)
    #     # else:
    #     #     print('i: Robot Movement number:',i)
    #     #     print('i: Pixel index number=',i)
    #     #     print('binary_img.item(i)',binary_img.item(i))
    #     #     # print(arduino.readline())             #read the serial data and print it as line
    #     #     # time.sleep(3)
    #     #     arduino_led.led_ON() 
    #     #     arduino_led.led_OFF() # Turns off after every movement - remove this for continuous LED ON
    #     #     time.sleep(0.5)

    #     if i == 0:
    #         # waypoints.append(copy.deepcopy(wpose))
    #         waypoints = []
    #         print('i=',i)
    #         print('wpose.position.y=',wpose.position.y)
    #     elif i % IMAGE_WIDTH == 0: # if i is a multiple of the image width, that means it should move to the next row
    #         print('Reached end of row, starting next row at index: ',i)
    #         # 3 is starting next row of pixels. 
    #         #So when robot finish position 2, it should go back to starting point & move down to start position 3
    #         # 0 1 2: once robot reaches 2, it needs to return to 3 basically reset and go down
    #         # 3 4 5
    #         wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
    #         wpose.position.y = y_start # same y-axis starting value
    #         waypoints.append(copy.deepcopy(wpose))
    #         print('i=',i)
    #         print('wpose.position.y=',wpose.position.y)

    #     # elif i == IMAGE_WIDTH*(IMAGE_HEIGHT-1):
    #     #     print('Reached end of row, starting next row at index: ',i)
    #     #     # 3 is starting next row of pixels. 
    #     #     #So when robot finish position 2, it should go back to starting point & move down to start position 3
    #     #     # 0 1 2: once robot reaches 2, it needs to return to 3 basically reset and go down
    #     #     # 3 4 5
    #     #     wpose.position.z -= MOTION_BOX_HEIGHT/IMAGE_HEIGHT
    #     #     wpose.position.y = y_start # same y-axis starting value
    #     #     waypoints.append(copy.deepcopy(wpose))
    #     #     print('i=',i)
    #     #     print('wpose.position.y=',wpose.position.y)

    #     else: # else keep incrementally moving horizontally across y-axis
    #             wpose.position.y += MOTION_BOX_WIDTH/IMAGE_WIDTH # Previously, MOTION_BOX_LENGTH/WIDTH = 3/9=1/3 m big jump
    #             waypoints.append(copy.deepcopy(wpose))
    #             print('i=',i)
    #             print('wpose.position.y=',wpose.position.y)
    #     # need to set bounds on how far it should go before starting the next row of pixels
    #     # wpose.position.z += MOTION_BOX_HEIGHT/WIDTH
    #     # waypoints.append(wpose)
    #     # rospy.loginfo(wpose)
    #     plan, fraction = rc.plan_cartesian_path(waypoints)
        
    #     # input(f"Cartesian Plan {i}: press <enter>") # uncomment this line if you want robot to run automatically

        
    #     rc.execute_plan(plan)




            

            # if binary_img.item(i) == 2 or 5 or 8: 
            #     # item 2,5,8 are the edge pixels. 
            #     # since we are going sequentially 0>1>2>3. 
            # Once light reaches edge pixel, it will move across the image
            #     arduino_led.led_OFF()

            # time.sleep(4)
            # arduino_led.led_OFF()
        # try:
        #     rospy.spin()
        # except KeyboardInterrupt:
        #     print("Shutting down")
        #     cv2.destroyAllWindows()

        
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
        rospy.init_node('RGB values')
        rospy.loginfo(">>RGB Values node successfully created")
        rgb_img = input_image.rgb

        RGB_values(rgb_img)
        # main()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    