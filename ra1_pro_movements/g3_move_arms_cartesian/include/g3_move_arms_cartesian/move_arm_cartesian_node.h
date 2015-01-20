#ifndef BAXTER_MOVE_ARM_CARTESIAN__H_
#define BAXTER_MOVE_ARM_CARTESIAN__H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace baxter_move_arms_cartesian
{
class BaxterMoveArmCartesianNode
{
public:
  BaxterMoveArmCartesianNode();
  ~BaxterMoveArmCartesianNode()
  {
  }
  ;

  bool init(std::string move_group_name);

  move_group_interface::MoveItErrorCode moveArmWithWaypoint(geometry_msgs::PoseStamped &waypoint, double &fraction);

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  void getRealWaypointFromWaypoint(geometry_msgs::PoseStamped &waypoint, geometry_msgs::PoseStamped &destination_waypoint);

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;

  //moveIt! display
  ros::Publisher display_publisher_pub_;
};
}

#endif
