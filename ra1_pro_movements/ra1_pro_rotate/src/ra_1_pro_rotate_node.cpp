#include <ra1_pro_rotate/ra_1_pro_rotate_node.h>

namespace ra1_pro_rotate
{
Ra1ProRotateNode::Ra1ProRotateNode() :
    nh_(), tf_listener_(nh_)
{
  max_planning_time_ = 180.0;
  num_planning_attempts_ = 100;
}

void Ra1ProRotateNode::init()
{
  ros::ServiceServer service_ = nh_.advertiseService("rotate_angle", &Ra1ProRotateNode::handleRotate, this);

  move_group_ = boost::make_shared<move_group_interface::MoveGroup>("arm");
  nh_.param<double>("max_planning_time", this->max_planning_time_, 180.0);
  nh_.param<int>("num_planning_attempts", this->num_planning_attempts_, 100);

  display_publisher_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  joint_state_srv_ = nh_.serviceClient<ra1_pro_msgs::ReturnJointStates>("robot/joint_states_filtered");
}

double Ra1ProRotateNode::getCurrentJointPosition(std::string &joint_name)
{
  ra1_pro_msgs::ReturnJointStates srv;
  srv.request.name = joint_name;
  if (joint_state_srv_.call(srv))
    if (srv.response.found != 0)
      return srv.response.position;
    else
      ROS_ERROR("Joint not found");
  else
    ROS_ERROR("Failed to call service");

  return -100.0;
}

move_group_interface::MoveItErrorCode Ra1ProRotateNode::rotateArmWithAngle(std::string &joint_name, double &angle,
    double &duration)
{

  ROS_INFO("[Ra1ProRotateNode::rotateArmWithAngle]: got angle... start planning");

  double current_position = getCurrentJointPosition(joint_name);
  if (current_position < -10.0)
  {
    ROS_ERROR_STREAM("joint name not found " << current_position);
    return 0;
  }

  double new_position = current_position + angle;
  if (new_position < -M_PI || new_position > M_PI)
  {
    ROS_ERROR_STREAM("new joint position out of [-Pi  Pi]: " << new_position);
    return 0;
  }

  ROS_INFO_STREAM("current position of " << joint_name << " is " << current_position);

  moveit_msgs::RobotTrajectory trajectory;

  std_msgs::Header header;

  header.frame_id = "/base";
  header.stamp = ros::Time::now();
  header.seq = 1;

  // joint names
  trajectory.joint_trajectory.joint_names.push_back(joint_name);
  trajectory.joint_trajectory.header = header;
  trajectory.joint_trajectory.points.resize(1);

  // positions
  trajectory.joint_trajectory.points[0].positions.resize(1);
  trajectory.joint_trajectory.points[0].positions[0] = new_position;

  // Velocities
  trajectory.joint_trajectory.points[0].velocities.resize(1);
  trajectory.joint_trajectory.points[0].velocities[0] = 0.0;

  trajectory.joint_trajectory.points[0].time_from_start = ros::Duration(duration);

  // execute plan
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;
  move_group_interface::MoveItErrorCode error_code = move_group_->execute(plan);

  sleep(0.5);
  ROS_INFO("[Ra1ProRotateNode::moveArmWithWaypoints]: done");

  return error_code;
}

bool Ra1ProRotateNode::handleRotate(ra1_pro_msgs::RotateAngle::Request &req, ra1_pro_msgs::RotateAngle::Response &res)
{
  res.error = rotateArmWithAngle(req.joint_name, req.delta, req.duration);
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotate_arm_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ra1_pro_rotate::Ra1ProRotateNode rotate_node;
  rotate_node.init();

  ROS_INFO("Rotate joint server ready!");
  ros::spin();

  return 0;
}
