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
#include <actionlib/server/simple_action_server.h>
#include <tf2_eigen/tf2_eigen.h>

// ROS Messages
#include <sensor_msgs/JointState.h>
#include <light_painting/SimpleMoveRequestAction.h>

// Descartes Tools
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>



// ------------------------
// Descartes Wrapper Class
// ------------------------

class SimpleMover {

  protected:

    /* ROS NodeHandle */
    ros::NodeHandle nh_;

    /* Setup Action Server */
    actionlib::SimpleActionServer<light_painting::SimpleMoveRequestAction> as_;
    std::string action_name_;

    /* Action Messages */
    light_painting::SimpleMoveRequestFeedback feedback_;
    light_painting::SimpleMoveRequestResult result_;

    /* Joint State */
    sensor_msgs::JointStateConstPtr current_state_ptr;


  public:

    /* Public Variables */
    const std::string world_frame = "base_link";
    const std::string tcp_frame = "tool0";

    /* Public Function Declarations */
    bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);   

    /* Constructor */
    SimpleMover(const std::string robot_group_name) : 
      as_(nh_, "move_descartes_line", boost::bind(&SimpleMover::MoveRequest, this, _1), false),
      action_name_("move_descartes_line")
    {  

      // Init ROS Action Server
      as_.start();

      // Init Descartes Robot Model
      descartes_core::RobotModelPtr model_setup (new descartes_moveit::IkFastMoveitStateAdapter());
      model = model_setup;

      // Robot model description name on ROS Parameter Server
      const std::string robot_description = "robot_description";

      // Initialize model
      if (!model->initialize(robot_description, robot_group_name, world_frame, tcp_frame))
      {
        ROS_INFO("Could not initialize robot model");
      }
      model->setCheckCollisions(true);

      // Descartes: Setup DensePlanner
      if (!planner.initialize(model))
      {
        ROS_ERROR("Failed to initialize planner");
      }

      // User Feedback
      ROS_INFO("SimpleMove server initialized and ready.");
    }

    ~SimpleMover(void) {}


    /* New Move Request */
    int MoveRequest(const light_painting::SimpleMoveRequestGoalConstPtr &message)
    {

      // Deconstruct action request pose in message into local variable
      Eigen::Isometry3d pose_goal;
      tf2::convert(message->goal, pose_goal);

      // Lookup current pose
      Eigen::Isometry3d pattern_origin = CalcCurrentPose();

      // Generate path plan starting from current pose
      std::vector<descartes_core::TrajectoryPtPtr> points = makeStraightPath(pattern_origin, pose_goal, message->duration);

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
      nh_.getParam("controller_joint_names", names);

      trajectory_msgs::JointTrajectory joint_solution;
      joint_solution.joint_names = names;
      
      const static double default_joint_vel = 0.5;  // Unit:[rad/s]  Define a default velocity (ie. first point in path) 
      if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
      {
        ROS_ERROR("Unable to convert Descartes trajectory to joint points");
        return -5;
      }


      // Send ROS trajectory to the robot action controller & report result
      result_.complete = executeTrajectory(joint_solution);
      as_.setSucceeded(result_);

      if (!result_.complete)
      {
        ROS_ERROR("Could not execute trajectory!");
        return -6;
      }


      // Wait till user kills the process (Control-C)
      ROS_INFO("SimpleMover: Done!");
      return 0;
    }


    /* Get Current Pose */
    Eigen::Isometry3d CalcCurrentPose(const std::string jointTopic = "/joint_states")
    {
      current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(jointTopic, ros::Duration(1));
      Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
      
      model->getFK(current_state_ptr->position, pattern_origin);

      return pattern_origin;
    }


  private:

    /* Private Variables */
    descartes_planner::DensePlanner planner;
    descartes_core::RobotModelPtr model;

    /* Private Function Declarations */
    descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt);
    descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);
    descartes_core::TrajectoryPtPtr makeJointPoint(const std::vector<double> &joints, double dt);
    std::vector<descartes_core::TrajectoryPtPtr> makeStraightPath(
      Eigen::Isometry3d pattern_start = Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d pattern_end = Eigen::Isometry3d::Identity(),
      int duration = 10
    );
};





// ----------------------
// Main
// ----------------------

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "simple_mover_server");

  // Keep ROS Node & Supporting Processes Alive (single thread)
  ros::AsyncSpinner spinner (2);
  spinner.start();

  // Init SimpleMover Class
  SimpleMover sm("mh5l");

  // Spin to keep server alive
  ros::waitForShutdown();

  return 0;
}



// ----------------------
// Support Functions
// ----------------------

descartes_core::TrajectoryPtPtr SimpleMover::makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr SimpleMover::makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr SimpleMover::makeJointPoint(const std::vector<double> &joints, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new JointTrajectoryPt(joints, TimingConstraint(dt)) );
}



// ----------------------
// Generate Path
// ----------------------

std::vector<descartes_core::TrajectoryPtPtr> SimpleMover::makeStraightPath(Eigen::Isometry3d pattern_start, Eigen::Isometry3d pattern_end, int duration)
{
  // Extract Pose Position Start-End Differences
  Eigen::Isometry3d pattern_diff = Eigen::Isometry3d::Identity();
  pattern_diff.translation() =  pattern_end.translation() - pattern_start.translation();

  // Path Settings
  const static double num_steps = 50;
  const double time_between_points = duration / num_steps;

  ROS_INFO_STREAM("SimpleMover Request Recieved. Constructing " << num_steps << " intermediate points. Time (sec) between points: " << time_between_points);

  // Generate straight line path
  EigenSTL::vector_Isometry3d pattern_poses;
  for (double i = 0.0; i <= 1.0; i = i + 1.0/num_steps)
  {
    // Grab Start Rotation
    Eigen::Isometry3d pose = pattern_start;

    // Cartesian Difference    
    pose.translation() = pattern_start.translation() + pattern_diff.translation() * i;
    pattern_poses.push_back(pose);
  }

  // Ensure first trajectory point is at exact start
  descartes_core::TrajectoryPtPtr pt = makeJointPoint(current_state_ptr->position, 0.01);

  std::vector<descartes_core::TrajectoryPtPtr> result;
  result.push_back(pt);

  // Assemble path as list of Descartes Points
  for (const auto& pose : pattern_poses)
  {
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint( pose, time_between_points);
    //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pose, time_between_points);
    result.push_back(pt);
  }

  return result;
}



// ----------------------
// Function Definitions
// ----------------------

bool SimpleMover::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("SimpleMover: Could not connect to action server");
    return false;
  }

  ROS_INFO("SimpleMover: Sent trajectory to motion server!");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(2.0);
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
