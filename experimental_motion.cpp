/**
 *  @file    motion.cpp
 *  @author  Johannes Offner
 *  @date    8/28/2017
 *  @version 1.0
 *
 *  @brief Roboy 2.0, calculate Inverse Kinematics, experimental publisher
 *
 *  @section DESCRIPTION
 *
 * This is experimental publisher node, that calculates FK first to get a valid position
 * and visualizes the moement for a few seconds.
 * Afterwards it calculates IK for "robot_description" if possible nd visualizes the movement 
 * in Rviz. Finally the joint trajectory will be published.
 *
 *
 */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "std_msgs/String.h"

#include <boost/scoped_ptr.hpp>

using namespace std;

int main(int argc, char** argv)
{
	
  // +************************************************** INITIALIZE********************************************************************
  // +*********************************************************************************************************************************
	
  ros::init(argc, argv, "motion");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = robot_model->getJointModelGroup("leg");
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
  
  // Get Initial Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  // +*********************************************** Forward Kinematics***************************************************************
  // +*********************************************************************************************************************************
  /* We output here the init position of the end effector (the transformation) after FK */
  const Eigen::Affine3d &end_effector_state_init = kinematic_state->getGlobalLinkTransform("pabi_legs__link_0_0");
  ROS_INFO_STREAM("Init_Translation: " << end_effector_state_init.translation());
  ROS_INFO_STREAM("Init_Rotation: " << end_effector_state_init.rotation());

  /* Sleep a little to allow time to startup rviz, etc. */
  ROS_INFO_STREAM("Starting RViz...");
  ros::WallDuration sleep_time(10.0);
  sleep_time.sleep();

  ROS_INFO("Setting up goal angles...");
  joint_values[0] = -0.15;
  joint_values[1] = 0.2;
  
  /* Calculating FK */
  robot_state::RobotState goal_state(robot_model);
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);


  /* We output here the final position of the end effector (the transformation) after FK */
  ROS_INFO("Forward Kinematics");
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("pabi_legs__link_0_0");
  ROS_INFO_STREAM("FK End Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("FK End Rotation: " << end_effector_state.rotation());
  
  
  // *************************************************** PLANNING INIT ****************************************************************
  
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
  
  /* Definitions */
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = "leg";
  planning_interface::PlanningContextPtr context;
  
  // ************************************************ FK VISUALIZATION ****************************************************************
  req.goal_constraints.push_back(joint_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the FK trajectory");
  moveit_msgs::MotionPlanResponse response;
  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  /* Now you should see two planned trajectories in series*/
  display_publisher.publish(display_trajectory);
  sleep_time.sleep();
  ROS_INFO("STOPPING VISUALIZATION...");
  
  // ********************************************** INVERSE KINEMATICS ************************************************************
  // +*********************************************************************************************************************************
  kinematic_state->setToDefaultValues();
  const Eigen::Affine3d &end_effector_state_ik_start = kinematic_state->getGlobalLinkTransform("pabi_legs__link_0_0");
  ROS_INFO_STREAM("Translation before IK: " << end_effector_state_ik_start.translation());
  ROS_INFO_STREAM("Rotation before IK: " << end_effector_state_ik_start.rotation());
  
  
  /* At the moment the FK position has to be fed in manually into here. TODO: read it out and set the values as IK pose parameters */
  // 0.03394;  -0.17347;   -0.37346;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.position.x = 0.03394;
  pose.pose.position.y = -0.17347;
  pose.pose.position.z = -0.37346;
  pose.pose.orientation.w = 1.0;;


  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.1);
  
  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("pabi_legs__link_0_0", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  
  // ************************************************ CALCULATING IKFAST *************************************************************
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  
  // ************************************************ VISUALIZE *********************************************************************
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the IK trajectory");
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  
  sleep_time.sleep();
  ROS_INFO("Done");

  return 0;
}
