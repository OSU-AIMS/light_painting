#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2021, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: acbuynak


import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class moveManipulator(object):
    """moveManipulator Class"""
    def __init__(self, group):
        super(moveManipulator, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('node_moveManipulator', anonymous=True)

        # Setup Variables needed for Moveit_Commander
        self.object_name = ''
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = group
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.group_names = self.robot.get_group_names()

        self.move_group.set_planning_time(20)
        self.move_group.set_num_planning_attempts(40)

    def set_vel(self,max_vel):
        self.move_group.set_max_velocity_scaling_factor(max_vel)

    def set_accel(self,max_accel):
        self.move_group.set_max_acceleration_scaling_factor(max_accel)

    def lookup_pose(self):
        return self.move_group.get_current_pose(self.eef_link).pose

    def goto_all_zeros(self):
        self.goto_joint_posn([0, 0, 0, 0, 0, 0])

    def goto_named_target(self, target):
        ## Trajectory in JOINT space

        # Send action to move-to defined position
        self.move_group.set_named_target(target)
        self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(target, current_joints, 0.01)

    def goto_joint_posn(self,joint_goal):
        ## Trajectory in JOINT space

        # Motion command w/ residual motion stop
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def goto_Pose(self, pose_goal):
        """
            Set pose_goal.
            Plan and execute.
            Stop to prevent residual motion
            Clear goal from target list.
        """
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def goto_Pose_w_tolerance(self, pose_goal, joint_tol=0, position_tol=0, orientation_tol=0):
        """
            Set pose_goal.
            Plan and execute.
            Stop to prevent residual motion
            Clear goal from target list.
        """
        self.move_group.set_goal_joint_tolerance(joint_tol)
        self.move_group.set_goal_orientation_tolerance(position_tol)
        self.move_group.set_goal_position_tolerance(orientation_tol)

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def plan_cartesian_path(self,waypoints):
        ## Plan Cartesian Path to throw object

        # Specify a list of waypoints
        # waypoints = []

        # Example - Commented Out
        #wpose = self.move_group.get_current_pose().pose
        #wpose.position.z += scale * 0.1  # Move up (z)
        #wpose.position.x += scale * 0.8  # Forward (x)
        #waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 5 cm
        # which is why we will specify 0.05 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(),
                                        plan,
                                        velocity_scaling_factor=0.1)
        
        # For reducing robot speed: https://answers.ros.org/question/376226/how-to-decrease-the-speed-of-my-robot/
        # As of 3/21: seems to work.
        rospy.loginfo(plan)
        return plan, fraction
    def execute_plan(self, plan):
        ## Execute a Plan
        ## Use execute if you would like the robot to follow a plan that has already been computed:

        ##TODO: https://answers.ros.org/question/377150/attributeerror-tuple-object-has-no-attribute-serialize-after-attempting-to-execute-a-plan/
        # No Longer an issue^^
        self.move_group.execute(plan, wait=True)

