#!/usr/bin/env python3

from curses import baudrate
import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose


# Custom Scripts
import light_painting.io as arduino


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

 #### Arduino Communication Plan: ############
    # Send I/O to turn on light on arduino
    # publish to cpp node
    # then in Arduino script, 
    # subscribe to cpp node to turn on light
    # hopefully this works??
    # NVM: we can write arduino code using python
    # https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0
    


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