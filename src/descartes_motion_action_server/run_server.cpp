/*
 * Descartes ROS Action Server
 * Recieves a pose, plans cartesian path to pose, accepts motion.
 * 
 * Copyright (c) 2022, The Ohio State University
 * The Artificially Intelligent Manufacturing Systems Lab (AIMS)
 * Author: A.C. Buynak
 */

#include <iostream>

// ROS Tools
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// ROS Messages
#include <sensor_msgs/JointState.h>

// Descartes Tools
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>



// ----------------------
// Function Declarations
// ----------------------

// Build Path
std::vector<descartes_core::TrajectoryPtPtr> makeStraightPath(
    Eigen::Isometry3d pattern_start = Eigen::Isometry3d::Identity(),
    Eigen::Isometry3d pattern_end = Eigen::Isometry3d::Identity()
    );

// Send Motion Commands
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);



// ----------------------
// Main
// ----------------------

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "single_descartes_move");
  ros::NodeHandle nh;

  // Keep ROS Node & Supporting Processes Alive (single thread)
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // Robot model: kinematics, group name
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());
  const std::string robot_description = "robot_description";
  const std::string group_name = "mh5l";

  // Reference frames for planning
  const std::string world_frame = "base_link";
  const std::string tcp_frame = "eef_tcp";

  // Initialize model
  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }
  model->setCheckCollisions(true);


  // Get current pose
  const std::string jointTopic = "/joint_states";
  sensor_msgs::JointStateConstPtr current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(jointTopic, ros::Duration(2));

  Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
  model->getFK(current_state_ptr->position, pattern_origin);



  // Get goal pose
  // TODO: get from action server
  // TEST ONLY: modify current position in one axis
  Eigen::Isometry3d pattern_delta = Eigen::Isometry3d::Identity();
  pattern_delta.translation() = Eigen::Vector3d(-0.1, 0, 0);

  Eigen::Isometry3d pose_goal = pattern_origin * pattern_delta;


  // Generate path plan starting from current pose
  std::vector<descartes_core::TrajectoryPtPtr> points = makeStraightPath(pattern_origin, pose_goal);


  // Descartes: Setup DensePlanner
  descartes_planner::DensePlanner planner;
  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }


  // Descartes: Build Trajectory
  // build large kinematic graph
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }
  // search kinematic graph to build trajectory
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }


  // Assemble ROS JointTrajectory Message
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;
  
  const static double default_joint_vel = 0.5;  // Unit:[rad/s]  Define a default velocity (ie. first point in path) 
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }


  // Send ROS trajectory to the robot action controller
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}



// ----------------------
// Support Functions
// ----------------------

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}



// ----------------------
// Generate Path
// ----------------------

std::vector<descartes_core::TrajectoryPtPtr> makeStraightPath(Eigen::Isometry3d pattern_start, Eigen::Isometry3d pattern_end)
{
  // Extract Pose Position Start-End Differences
  Eigen::Isometry3d pattern_diff = Eigen::Isometry3d::Identity();
  pattern_diff.translation() =  pattern_end.translation() - pattern_start.translation();

  // Path Settings
  const static double num_steps = 5;
  const static double time_between_points = 0.01;

  // Generate straight line path
  EigenSTL::vector_Isometry3d pattern_poses;
  for (double i = 0.0; i < 1.0; i = i + 1.0/num_steps)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = pattern_start.translation() + pattern_diff.translation() * i;
    pattern_poses.push_back(pose);
  }

  // Ensure first trajectory point is at exact start
  std::vector<descartes_core::TrajectoryPtPtr> result;
  descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_start, time_between_points);
  result.push_back(pt);

  // Assemble path as list of Descartes Points
  for (const auto& pose : pattern_poses)
  {
    // descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_start * pose, time_between_points);
    descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_start * pose, time_between_points);
    result.push_back(pt);
  }

  return result;
}



// ----------------------
// Function Definitions
// ----------------------

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  ROS_INFO("Connected to action server!");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(2.0);
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
