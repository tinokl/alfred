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

#include "ra1_pro_msgs/MovePose.h"
#include "ra1_pro_msgs/MoveFixPose.h"

namespace ra1_pro_move
{

class RA1ProMove
{

public:

  RA1ProMove();

  ~RA1ProMove(){};

  void init();
  bool handleSetFixPose(ra1_pro_msgs::MoveFixPose::Request &req, ra1_pro_msgs::MoveFixPose::Response &res);
  bool handleSetArmPose(ra1_pro_msgs::MovePose::Request &req, ra1_pro_msgs::MovePose::Response &res);
  bool findPlan(moveit::planning_interface::MoveGroup::Plan &plan);
  bool moveArmToPose(geometry_msgs::Pose &new_pose);
  bool moveToRandomPose();

  ros::Publisher display_publisher_pub_;
  ros::Publisher robot_state_publisher_;

private:

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;

  ros::NodeHandle nh_;
  ros::ServiceServer pose_service_;
  ros::ServiceServer fix_pose_service_;

};
}

#endif
