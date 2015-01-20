#include <ros/ros.h>
#include <g3_msgs/manage_collision_scene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_manager_service");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<g3_msgs::manage_collision_scene>("manage_collision_scene");

  // EGG
/*
  g3_msgs::manage_collision_scene srv;
  srv.request.id = "egg";
  srv.request.pose.header.frame_id = "right_hand";
  srv.request.pose.header.stamp = ros::Time::now();
  srv.request.pose.pose.position.z = 0.12;
  srv.request.pose.pose.orientation.w = 1.0;
  srv.request.operation = srv.request.ATTACH;
*/

  // POT

  g3_msgs::manage_collision_scene srv;
  srv.request.id = "pot";
  srv.request.pose.header.frame_id = "base";
  srv.request.pose.header.stamp = ros::Time::now();
  srv.request.pose.pose.position.x = 1.0;
  srv.request.pose.pose.position.z = 0.04;
  srv.request.pose.pose.orientation.w = 1.0;
  srv.request.operation = srv.request.ADD;


  ros::service::waitForService("manage_collision_scene");

  if (client.call(srv))
  {
    ROS_INFO("service manage collision scene called");
  }
  else
  {
    ROS_ERROR("service call manage collision scene failed");
  }
  return 0;
}
