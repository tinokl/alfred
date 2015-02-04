#ifndef COLLISION_SCENE
#define COLLISION_SCENE

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ra1_pro_msgs/ManageCollisionScene.h>

namespace collision_scene
{
class CollisionScene
{

public:

  CollisionScene();
  virtual ~CollisionScene();

  void init();

  bool manageCollisionScene(ra1_pro_msgs::ManageCollisionScene::Request &req,
      ra1_pro_msgs::ManageCollisionScene::Response &res);
  moveit_msgs::CollisionObject getCollisionObject(std::string object,
      std::string target_frame, geometry_msgs::Pose pose);

  void attachCO(moveit_msgs::CollisionObject object);
  void detachCO(moveit_msgs::CollisionObject object);
  void removeCO(moveit_msgs::CollisionObject object);
  void allowCollision(moveit_msgs::CollisionObject object);
  void denyCollision(moveit_msgs::CollisionObject object);
  void addCO(moveit_msgs::CollisionObject object);

  void addEnvironment();

private:

  CollisionScene(const CollisionScene &src);

  ros::NodeHandle nh_;
  ros::ServiceServer server_;

  bool once_;
  collision_detection::AllowedCollisionMatrix acm_;

  ros::Publisher collision_object_publisher_;
  ros::Publisher attached_object_publisher_;
  ros::Publisher planning_scene_diff_publisher;
};

}
#endif
