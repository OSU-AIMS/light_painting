#!/usr/bin/env python3

import rospy
from robot_control import * 
import copy
from geometry_msgs.msg import Pose

def main():
    rospy.init_node('light_painting', anonymous=False)
    rc= moveManipulator('mh5l')
    rc.set_vel(0.1)
    rc.set_accel(0.1)

    rc.goto_all_zeros()

    waypoints = []
    scale = 1

    wpose = rc.move_group.get_current_pose().pose
    wpose.position.x += scale * 0.8 
    waypoints.append(wpose)

    plan = rc.plan_cartesian_path(waypoints)

    input("Cartesian Plan: press <enter>")
    # input("Cartesian Plan: press <enter>")
    rc.execute_plan(plan)

    input("All zeros: press <enter>")
    # input("All zeros: press <enter>")

    rc.goto_all_zeros()

if __name__ == '__main__':
    main()