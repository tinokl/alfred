#include <ra1_pro_move_cartesian/ra1_pro_move_cartesian_node.h>
#include "ra1_pro_msgs/MoveCartesian.h"

namespace ra1_pro_move_cartesian
{
RA1ProMoveCartesianNode::RA1ProMoveCartesianNode() :
    nh_(), tf_listener_(nh_)
{
  max_planning_time_ = 180.0;
  num_planning_attempts_ = 100;
}

bool RA1ProMoveCartesianNode::init(std::string move_group_name)
{
  move_group_ = boost::make_shared<move_group_interface::MoveGroup>(move_group_name);
  nh_.param<double>("max_planning_time", this->max_planning_time_, 180.0);
  nh_.param<int>("num_planning_attempts", this->num_planning_attempts_, 100);

  display_publisher_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  return true;
}

void RA1ProMoveCartesianNode::getRealWaypointFromWaypoint(geometry_msgs::PoseStamped &waypoint,
    geometry_msgs::PoseStamped &real_waypoint)
{
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  geometry_msgs::PoseStamped new_pose_transformed;

  try
  {
    std::string error_str;

    // transform into new frame
    if (tf_listener_.waitForTransform(waypoint.header.frame_id.c_str(), current_pose.header.frame_id, current_pose.header.stamp,
        ros::Duration(5.0), ros::Duration(0.01), &error_str))
      tf_listener_.transformPose(waypoint.header.frame_id.c_str(), current_pose, new_pose_transformed);
    else
      throw tf::ConnectivityException("tf_listener error_str: " + error_str);

    // add movements:
    //   position
    //   orientation (untouched)
    new_pose_transformed.pose.position.x += waypoint.pose.position.x;
    new_pose_transformed.pose.position.y += waypoint.pose.position.y;
    new_pose_transformed.pose.position.z += waypoint.pose.position.z;

    //transform back
    real_waypoint.header = current_pose.header;

    if (tf_listener_.waitForTransform(real_waypoint.header.frame_id.c_str(), new_pose_transformed.header.frame_id,
        new_pose_transformed.header.stamp, ros::Duration(10.0), ros::Duration(0.01), &error_str))
      tf_listener_.transformPose(real_waypoint.header.frame_id.c_str(), new_pose_transformed, real_waypoint);
    else
      throw tf::ConnectivityException("tf_listener error_str: " + error_str);

  }
  catch (...)
  {
    ROS_ERROR("[RA1ProMoveCartesianNode] caught any exception...");
    throw false;
  }
}

move_group_interface::MoveItErrorCode RA1ProMoveCartesianNode::moveArmWithWaypoint(geometry_msgs::PoseStamped &waypoint,
    double &fraction)
{
  ROS_INFO("[RA1ProMoveCartesianNode::moveArmWithWaypoints]: got waypoint...start planning of cartesian path");

  // get real waypoint
  geometry_msgs::PoseStamped real_waypoint;
  getRealWaypointFromWaypoint(waypoint, real_waypoint);

  // create plan
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(real_waypoint.pose);
  moveit_msgs::RobotTrajectory trajectory;
  fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  //ROS_ERROR_STREAM("Trajectory: " << trajectory);
  ROS_INFO("[RA1ProMoveCartesianNode::moveArmWithWaypoints]: Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  if (fraction < 0.01)
    return moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

  // sleep to give Rviz time to visualize the plan.
  sleep(0.5);

  // execute plan
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;
  move_group_interface::MoveItErrorCode error_code = move_group_->execute(plan);

  sleep(1.0);
  ROS_INFO("[RA1ProMoveCartesianNode::moveArmWithWaypoints]: done");

  return error_code;
}

}

bool handleSetMoveCartesian(ra1_pro_msgs::MoveCartesian::Request &req, ra1_pro_msgs::MoveCartesian::Response &res)
{
  std::string move_group_name(req.move_group);
  geometry_msgs::PoseStamped waypoint(req.waypoint);

  ROS_INFO_STREAM("New waypoint for movegroup '" << move_group_name << "'");

  ra1_pro_move_cartesian::RA1ProMoveCartesianNode move_group;
  if (!move_group.init(move_group_name))
  {
    res.error = move_group_interface::MoveItErrorCode::INVALID_GROUP_NAME;
    return true;
  }

  double fraction = 0.0;
  res.error = move_group.moveArmWithWaypoint(waypoint, fraction);
  res.fraction = fraction;
  sleep(1.0);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_cartesian_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::ServiceServer service1 = nh.advertiseService("move_cartesian", handleSetMoveCartesian);
  ROS_INFO("Ready to get a new waypoint.");
  ros::spin();

  return 0;
}
