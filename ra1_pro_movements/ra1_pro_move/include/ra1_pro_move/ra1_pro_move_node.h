#ifndef RA1_PRO_MOVE__H_
#define RA1_PRO_MOVE__H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace ra1_pro_move
{

class RA1ProMove
{

public:

  RA1ProMove(std::string move_group_name);

  ~RA1ProMove(){};

  bool moveArmToPose(geometry_msgs::Pose &new_pose);
  bool moveToRandomPose();

  //moveit display
  ros::Publisher display_publisher_pub_;

  ros::Publisher robot_state_publisher_;

private:

  ros::NodeHandle nh_;

  bool findPlan(moveit::planning_interface::MoveGroup::Plan &plan);

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;



};
}

#endif
