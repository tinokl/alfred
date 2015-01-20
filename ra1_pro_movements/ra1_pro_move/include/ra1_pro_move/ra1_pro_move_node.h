#ifndef RA1_PRO_MOVE__H_
#define RA1_PRO_MOVE__H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>

namespace ra1_pro_move
{

class RA1ProMove
{

public:

  RA1ProMove(std::string move_group_name);

  ~RA1ProMove(){};

  bool moveArmToPose(geometry_msgs::Pose &new_pose);

private:

  ros::NodeHandle nh_;

  bool findPlan(moveit::planning_interface::MoveGroup::Plan &plan);

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;

  //moveit display
  ros::Publisher display_publisher_pub_;

};
}

#endif
