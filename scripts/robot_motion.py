#!/usr/bin/env python3

import rospy
from robot_control import * 
# import copy
from geometry_msgs.msg import Pose

# For Arduino Control
import serial
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)

# Custom Scripts
from test import py_to_ino_LIGHT_ON_OFF_test as arduino_led


def main():
    rospy.init_node('light_painting', anonymous=False)
    rospy.loginfo(">> light_painting Node sucessfully created")

    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    rc.goto_all_zeros() 

    waypoints = []
    scale =0.5

    wpose = rc.move_group.get_current_pose().pose
    wpose.position.x += scale * 0.1 
    # Robot motions don't match RVIZ Axis for EEF TCP
    # wpose.position.y: twists the arm about the blue axis
    # wpose.position.x: moves robot forward
    # wpose.position.z: moves robot vertically up
    waypoints.append(wpose)

    plan, fraction = rc.plan_cartesian_path(waypoints)
    # Added fraction because of this Github issue: 
    # https://github.com/ros-planning/moveit/issues/709

    input("Cartesian Plan: press <enter>")
    # for Python 2.7: raw_input("Cartesian Plan: press <enter>")
    rc.execute_plan(plan)

    print(arduino.readline())             #read the serial data and print it as line
    print("Enter 1 to ON LED and 0 to OFF LED")
    user_input = input()                  #waits until user enters data

    arduino_led.control_led(user_input)     # call to turn LED on/off (as of 3/9: currently requires user input)


 #### Arduino Communication Plan: ############
    


    input("All zeros: press <enter>")
    # for Python 2.7: raw_input("All zeros: press <enter>")

    rc.goto_all_zeros()

'''
Plan for integrating Arduino with robot: 
- Send I/O to arduino board via line of code (instead of pushbutton) Send Digital High maybe?
- This turns light on/off at certain points
- Move robot as normal in script. 
'''


'''
for Arduino Control:
Install Rosserial Library into Arduino
Rosserial Arduino Examples: https://github.com/ros-drivers/rosserial/tree/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples
'''

if __name__ == '__main__':
    main()