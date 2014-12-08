#include <ra1_pro_move/ra1_pro_move_node.h>
#include "ra1_pro_msgs/MovePose.h"

namespace ra1_pro_move
{

RA1ProMove::RA1ProMove(std::string move_group_name) : nh_()
{
  move_group_ = boost::make_shared<move_group_interface::MoveGroup>(move_group_name);
  nh_.param<double>("max_planning_time", this->max_planning_time_, 10.0);
  nh_.param<int>("num_planning_attempts", this->num_planning_attempts_, 10);

  display_publisher_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

bool RA1ProMove::findPlan(moveit::planning_interface::MoveGroup::Plan &plan)
{
  for (uint32_t n_trail = 0; n_trail < num_planning_attempts_; ++n_trail)
  {
    ROS_INFO("[RA1ProMove::findPlan]: Search new plan");
    if (move_group_->plan(plan))
      return true;
  }

  return false;
}

bool RA1ProMove::moveArmToPose(geometry_msgs::Pose &new_pose)
{
  move_group_->setPlanningTime(max_planning_time_);
  move_group_->setPoseTarget(new_pose);

  moveit::planning_interface::MoveGroup::Plan move_plan;
  if (findPlan(move_plan))
  {
    ROS_INFO("[RA1ProMove::moveArmToPose]: Visualizing plan");

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = move_plan.start_state_;
    display_trajectory.trajectory.push_back(move_plan.trajectory_);
    display_publisher_pub_.publish(display_trajectory);

    move_group_->execute(move_plan);
    sleep(1.0);
    ROS_INFO("[RA1ProMove::moveArmToPose]: done");
    return true;
  }
  ROS_INFO("[RA1ProMove::moveArmToPose]: Visualizing plan FAILED");
  ROS_ERROR("[RA1ProMove::moveArmToPose]: No motion plan found");
  return false;
}

}

bool handleSetArmPose(ra1_pro_msgs::MovePose::Request &req, ra1_pro_msgs::MovePose::Response &res)
{
  ra1_pro_move::RA1ProMove arm("arm");

  res.error = res.NO_ERROR;

  ROS_INFO_STREAM("New pose for arm: " << req.pose);

  if (!arm.moveArmToPose(req.pose))
    res.error = res.ERROR;

  sleep(1.0);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ra1_pro_move");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("move_pose", handleSetArmPose);
  ROS_INFO("Ready to get arm poses");
  ros::spin();

  return 0;
}
