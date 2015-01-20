#include <ra1_pro_collision_scene/collision_scene_node.h>

namespace collision_scene
{

CollisionScene::CollisionScene()
{}

CollisionScene::~CollisionScene()
{}

void CollisionScene::init()
{
	ros::Rate loop_rate(10);
	nh_ = ros::NodeHandle();
	//tf_ = new tf::TransformListener(ros::Duration(2.0));
	//boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	//planning_scene_monitor_ = new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf);
	server_ = nh_.advertiseService("manage_collision_scene", &CollisionScene::manageCollisionScene, this);

	//planning_scene_ = planning_scene_monitor_->getPlanningScene();
	//acm_ = planning_scene_->getAllowedCollisionMatrix();

    ROS_INFO_STREAM("Init Collision Scene");

	collision_object_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    while(collision_object_publisher_.getNumSubscribers() < 1)
    {
      ROS_INFO_STREAM("Waiting for 'collision_object_publisher' subsribers");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }

	attached_object_publisher_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    while(attached_object_publisher_.getNumSubscribers() < 1)
    {
      ROS_INFO_STREAM("Waiting for 'attached_object_publisher' subsribers");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }

    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO_STREAM("Waiting for 'planning_scene_diff_publisher' subsribers");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }

	addEnvironment();
}


bool CollisionScene::manageCollisionScene(g3_msgs::manage_collision_scene::Request &req,
                                          g3_msgs::manage_collision_scene::Response &res)
{
  std::string object_id = req.id;
  std::string target_frame = req.pose.header.frame_id;
  geometry_msgs::Pose pose = req.pose.pose;

  moveit_msgs::CollisionObject object = getCollisionObject(object_id, target_frame, pose);

  switch (req.operation)
  {
    case g3_msgs::manage_collision_sceneRequest::ADD:
      ROS_DEBUG("ADD mode:");
      addCO(object);
      break;
    case g3_msgs::manage_collision_sceneRequest::ATTACH:
      ROS_DEBUG("ATTACH mode:");
      attachCO(object);
      break;
    case g3_msgs::manage_collision_sceneRequest::DETACH:
      ROS_DEBUG("DETACH mode:");
      detachCO(object);
      break;
    case g3_msgs::manage_collision_sceneRequest::ALLOW_COLLISION:
      ROS_DEBUG("ALLOW_COLLISION mode:");
      allowCollision(object);
      break;
    case g3_msgs::manage_collision_sceneRequest::DENY_COLLISION:
      ROS_DEBUG("DENY_COLLISION mode:");
      denyCollision(object);
      break;
    case g3_msgs::manage_collision_sceneRequest::REMOVE:
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
	ROS_INFO("Adding the Environment to the planning scene");

	float kitchen_offset_x = -0.1335;
	float kitchen_offset_y = 0.0;
	float kitchen_offset_z = 0.0;

	// Kitchen Plate
	shape_msgs::SolidPrimitive kitchen_plate;
	kitchen_plate.type = kitchen_plate.BOX;
	kitchen_plate.dimensions.resize(3);
	kitchen_plate.dimensions[0] = 0.697;
	kitchen_plate.dimensions[1] = 3.46;
	kitchen_plate.dimensions[2] = 0.912;

	geometry_msgs::Pose plate_pose;
	plate_pose.orientation.w = 1.0;
	plate_pose.position.x = 1.0135 + kitchen_offset_x;
	plate_pose.position.y = 0.0 + kitchen_offset_y;
	plate_pose.position.z = -0.456 + kitchen_offset_z;
	plate_pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject kitchen_plate_co;
	kitchen_plate_co.header.frame_id = "base";
	kitchen_plate_co.id = "kitchen_plate";

	kitchen_plate_co.primitives.push_back(kitchen_plate);
	kitchen_plate_co.primitive_poses.push_back(plate_pose);
	kitchen_plate_co.operation = kitchen_plate_co.ADD;

	// Kitchen Shelf
	shape_msgs::SolidPrimitive kitchen_shelf;
	kitchen_shelf.type = kitchen_shelf.BOX;
	kitchen_shelf.dimensions.resize(3);
	kitchen_shelf.dimensions[0] = 0.445;
	kitchen_shelf.dimensions[1] = 3.46;
	kitchen_shelf.dimensions[2] = 0.917;

	geometry_msgs::Pose shelf_pose;
	shelf_pose.orientation.w = 1.0;
	shelf_pose.position.x = 1.10 + kitchen_offset_x;
	shelf_pose.position.y = 0.0 + kitchen_offset_y;
	shelf_pose.position.z = 1.05 + kitchen_offset_z;
	shelf_pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject kitchen_shelf_co;
	kitchen_shelf_co.header.frame_id = "base";
	kitchen_shelf_co.id = "kitchen_shelf";

	kitchen_shelf_co.primitives.push_back(kitchen_shelf);
	kitchen_shelf_co.primitive_poses.push_back(shelf_pose);
	kitchen_shelf_co.operation = kitchen_shelf_co.ADD;

	// Kitchen Wall
	shape_msgs::SolidPrimitive kitchen_wall;
	kitchen_wall.type = kitchen_wall.BOX;
	kitchen_wall.dimensions.resize(3);
	kitchen_wall.dimensions[0] = 0.1;
	kitchen_wall.dimensions[1] = 3.46;
	kitchen_wall.dimensions[2] = 2.417;

	geometry_msgs::Pose wall_pose;
	wall_pose.orientation.w = 1.0;
	wall_pose.position.x = 1.41 + kitchen_offset_x;
	wall_pose.position.y = 0.0 + kitchen_offset_y;
	wall_pose.position.z = 0.27 + kitchen_offset_z;
	wall_pose.orientation.w = 1.0;

	moveit_msgs::CollisionObject kitchen_wall_co;
	kitchen_wall_co.header.frame_id = "base";
	kitchen_wall_co.id = "kitchen_wall";

	kitchen_wall_co.primitives.push_back(kitchen_wall);
	kitchen_wall_co.primitive_poses.push_back(wall_pose);
	kitchen_wall_co.operation = kitchen_wall_co.ADD;

	addCO(kitchen_plate_co);
	addCO(kitchen_shelf_co);
	addCO(kitchen_wall_co);

	ROS_INFO("Adding the Environment finished");
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
        ros::init(argc, argv, "collision_scene_init" );
        collision_scene::CollisionScene collision_scene;
        collision_scene.init();
        ros::spin();
    }
    catch( ... )
    {
        ROS_ERROR_NAMED("collision_scene_init","Unhandled exception!");
        return -1;
    }

    return 0;
}
