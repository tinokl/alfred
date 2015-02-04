#include <ra1_pro_collision_scene/collision_scene_node.h>

namespace collision_scene
{

CollisionScene::CollisionScene() :
    once_(true)
{}

CollisionScene::~CollisionScene()
{}

void CollisionScene::init()
{
  ros::Rate loop_rate(0.1);
  nh_ = ros::NodeHandle();

  server_ = nh_.advertiseService("manage_collision_scene",
      &CollisionScene::manageCollisionScene, this);

  ROS_INFO_STREAM("Init Collision Scene");

  collision_object_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>(
      "collision_object", 1);

  attached_object_publisher_ = nh_.advertise<
      moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);

  planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>(
      "/move_group/monitored_planning_scene", 1);

  while (ros::ok())
  {
    addEnvironment();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool CollisionScene::manageCollisionScene(ra1_pro_msgs::ManageCollisionScene::Request &req,
                                          ra1_pro_msgs::ManageCollisionScene::Response &res)
{
  std::string object_id = req.id;
  std::string target_frame = req.pose.header.frame_id;
  geometry_msgs::Pose pose = req.pose.pose;

  moveit_msgs::CollisionObject object = getCollisionObject(object_id, target_frame, pose);

  switch (req.operation)
  {
    case ra1_pro_msgs::ManageCollisionSceneRequest::ADD:
      ROS_DEBUG("ADD mode:");
      addCO(object);
      break;
    case ra1_pro_msgs::ManageCollisionSceneRequest::ATTACH:
      ROS_DEBUG("ATTACH mode:");
      attachCO(object);
      break;
    case ra1_pro_msgs::ManageCollisionSceneRequest::DETACH:
      ROS_DEBUG("DETACH mode:");
      detachCO(object);
      break;
    case ra1_pro_msgs::ManageCollisionSceneRequest::ALLOW_COLLISION:
      ROS_DEBUG("ALLOW_COLLISION mode:");
      allowCollision(object);
      break;
    case ra1_pro_msgs::ManageCollisionSceneRequest::DENY_COLLISION:
      ROS_DEBUG("DENY_COLLISION mode:");
      denyCollision(object);
      break;
    case ra1_pro_msgs::ManageCollisionSceneRequest::REMOVE:
      ROS_DEBUG("REMOVE mode:");
      removeCO(object);
      break;
    default:
      ROS_ERROR("Something went totally wrong!");
      break;
  }

  return true;
}


void CollisionScene::addEnvironment()
{
  if (once_)
    ROS_INFO("Adding the Environment to the planning scene");

	// Desk
	shape_msgs::SolidPrimitive desk;
	desk.type = desk.BOX;
	desk.dimensions.resize(3);
	desk.dimensions[0] = 0.65;
	desk.dimensions[1] = 1.40;
	desk.dimensions[2] = 0.05;

	geometry_msgs::Pose desk_pose;
	desk_pose.orientation.w = 1.0;
	desk_pose.position.x = 0.18;
	desk_pose.position.y = -0.57;
	desk_pose.position.z = -0.025;
	desk_pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject desk_co;
	desk_co.header.frame_id = "base";
	desk_co.id = "desk";

	desk_co.primitives.push_back(desk);
	desk_co.primitive_poses.push_back(desk_pose);
	desk_co.operation = desk_co.ADD;

	// Walls
	shape_msgs::SolidPrimitive left_wall;
	left_wall.type = left_wall.BOX;
	left_wall.dimensions.resize(3);
	left_wall.dimensions[0] = 1.0;
	left_wall.dimensions[1] = 0.05;
	left_wall.dimensions[2] = 1.0;

	shape_msgs::SolidPrimitive back_wall;
	back_wall.type = back_wall.BOX;
	back_wall.dimensions.resize(3);
	back_wall.dimensions[0] = 0.05;
	back_wall.dimensions[1] = 1.0;
	back_wall.dimensions[2] = 1.0;

	geometry_msgs::Pose left_wall_pose;
	left_wall_pose.position.x = 0.10;
	left_wall_pose.position.y = 0.2;
	left_wall_pose.position.z = 0.45;
	left_wall_pose.orientation.w = 1.0;

	geometry_msgs::Pose back_wall_pose;
	back_wall_pose.position.x = -0.37;
	back_wall_pose.position.y = -0.30;
	back_wall_pose.position.z = 0.45;
	back_wall_pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject left_wall_co;
	moveit_msgs::CollisionObject back_wall_co;
	left_wall_co.header.frame_id = "base";
	back_wall_co.header.frame_id = "base";
	left_wall_co.id = "left_wall";
	back_wall_co.id = "back_wall";

	left_wall_co.primitives.push_back(left_wall);
	left_wall_co.primitive_poses.push_back(left_wall_pose);
	left_wall_co.operation = left_wall_co.ADD;

	back_wall_co.primitives.push_back(back_wall);
	back_wall_co.primitive_poses.push_back(back_wall_pose);
	back_wall_co.operation = back_wall_co.ADD;

	addCO(desk_co);
	addCO(left_wall_co);
	addCO(back_wall_co);

	if (once_)
	{
	  ROS_INFO("Adding the Environment finished");
	  once_ = false;
	}
}

moveit_msgs::CollisionObject CollisionScene::getCollisionObject(std::string object_id, std::string target_frame, geometry_msgs::Pose pose)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = target_frame;
  collision_object.id = object_id;
  shape_msgs::SolidPrimitive primitive;
  if (std::strcmp(object_id.c_str(),"ball") == 0)
  {
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.01;
    primitive.dimensions[1] = 0.01;
    primitive.dimensions[2] = 0.01;
  }
  else if (std::strcmp(object_id.c_str(), "pot") == 0)
  {
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.07;
    primitive.dimensions[1] = 0.24;
  }
  else
  {
	ROS_ERROR_STREAM("Unknown Collision Object, created a box!");
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.2;
	primitive.dimensions[1] = 0.2;
	primitive.dimensions[2] = 0.2;
  }

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  return collision_object;
}

void CollisionScene::addCO(moveit_msgs::CollisionObject object)
{
  collision_object_publisher_.publish(object);

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
}

void CollisionScene::attachCO(moveit_msgs::CollisionObject object)
{
  ROS_INFO_STREAM("Attach Collision Object: " << object.id);
  moveit_msgs::AttachedCollisionObject attach_object;
  attach_object.link_name = object.header.frame_id;
  attach_object.object = object;

  attached_object_publisher_.publish(attach_object);
}

void CollisionScene::detachCO(moveit_msgs::CollisionObject object)
{
  ROS_INFO_STREAM("Detach Collision Object: " << object.id);
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.link_name = object.header.frame_id;
  detach_object.object = object;
  detach_object.object.operation = object.REMOVE;
  attached_object_publisher_.publish(detach_object);
}

void CollisionScene::removeCO(moveit_msgs::CollisionObject object)
{
  ROS_INFO_STREAM("Remove Collision Object: " << object.id);

  object.operation = object.REMOVE;
  collision_object_publisher_.publish(object);
}

void CollisionScene::allowCollision(moveit_msgs::CollisionObject object)
{
	ROS_ERROR_STREAM("NOT SUPPORTED YET");
  //ROS_INFO_STREAM("Allow Collision with: " << object.id);
  //acm_.setEntry(object.id, true);
}

void CollisionScene::denyCollision(moveit_msgs::CollisionObject object)
{
	ROS_ERROR_STREAM("NOT SUPPORTED YET");
  //ROS_INFO_STREAM("Deny Collision with: " << object.id);
  //acm_.removeEntry(object.id);
}

}


int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "collision_scene_init");
    collision_scene::CollisionScene collision_scene;
    collision_scene.init();
    ros::spin();
  } catch (...)
  {
    ROS_ERROR_NAMED("collision_scene_init", "Unhandled exception!");
    return -1;
  }

  return 0;
}
