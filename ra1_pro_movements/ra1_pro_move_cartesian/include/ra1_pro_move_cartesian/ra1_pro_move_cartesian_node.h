#ifndef RA1_PRO_MOVE_CARTESIAN__H_
#define RA1_PRO_MOVE_CARTESIAN__H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "ra1_pro_msgs/MoveCartesian.h"

namespace ra1_pro_move_cartesian
{
class RA1ProMoveCartesianNode
{
public:
  RA1ProMoveCartesianNode();
  ~RA1ProMoveCartesianNode(){};

  void init();
  move_group_interface::MoveItErrorCode moveArmWithWaypoint(geometry_msgs::PoseStamped &waypoint, double &fraction);
  bool handleSetMoveCartesian(ra1_pro_msgs::MoveCartesian::Request &req, ra1_pro_msgs::MoveCartesian::Response &res);
  void getRealWaypointFromWaypoint(geometry_msgs::PoseStamped &waypoint, geometry_msgs::PoseStamped &destination_waypoint);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  tf::TransformListener tf_listener_;

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;
  ros::Publisher display_publisher_pub_;
};
}

#endif
