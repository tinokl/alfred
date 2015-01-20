#ifndef RA1_PRO_ROTATE__H_
#define RA1_PRO_ROTATE__H_

#include <ros/ros.h>

#include "ra1_pro_msgs/RotateAngle.h"
#include "ra1_pro_msgs/ReturnJointStates.h"

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <sensor_msgs/JointState.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace ra1_pro_rotate
{
class Ra1ProRotateNode
{
public:
  Ra1ProRotateNode();
  ~Ra1ProRotateNode()
  {
  }
  ;

  bool init(std::string move_group_name);

  move_group_interface::MoveItErrorCode rotateArmWithAngle(std::string &joint_name, double &angle, double &duration);

  double getCurrentJointPosition(std::string &joint_name);

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::ServiceClient joint_state_srv_;

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  double max_planning_time_;
  int32_t num_planning_attempts_;

  //moveIt! display
  ros::Publisher display_publisher_pub_;
};
}

#endif
