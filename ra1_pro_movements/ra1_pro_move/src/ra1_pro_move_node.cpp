#include <ra1_pro_move/ra1_pro_move_node.h>

namespace ra1_pro_move
{

RA1ProMove::RA1ProMove() :
    nh_()
{
  max_planning_time_ = 180.0;
  num_planning_attempts_ = 100;
}

void RA1ProMove::init()
{
  pose_service_ = nh_.advertiseService("move_pose", &RA1ProMove::handleSetArmPose, this);
  fix_pose_service_ = nh_.advertiseService("move_fix_pose", &RA1ProMove::handleSetFixPose, this);

  move_group_ = boost::make_shared<move_group_interface::MoveGroup>("arm");

  nh_.param<double>("max_planning_time", this->max_planning_time_, 180.0);
  nh_.param<int>("num_planning_attempts", this->num_planning_attempts_, 100);

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
  move_group_->setStartStateToCurrentState();
  move_group_->setApproximateJointValueTarget(new_pose, "wrist_s2");
  move_group_->allowReplanning(true);
  //move_group_->setPoseTarget(new_pose);

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

bool RA1ProMove::moveToRandomPose()
{
  move_group_->setPlanningTime(max_planning_time_);
  move_group_->setStartStateToCurrentState();
  move_group_->setRandomTarget();
  move_group_->allowReplanning(true);

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

bool RA1ProMove::handleSetArmPose(ra1_pro_msgs::MovePose::Request &req, ra1_pro_msgs::MovePose::Response &res)
{
  res.error = res.NO_ERROR;

  ROS_INFO_STREAM("New pose for arm: " << req.pose);

  if (!moveArmToPose(req.pose))
    res.error = res.ERROR;

  return true;
}

bool RA1ProMove::handleSetFixPose(ra1_pro_msgs::MoveFixPose::Request &req, ra1_pro_msgs::MoveFixPose::Response &res)
{
  /*
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(kinematic_model));
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  robot_state->setToDefaultValues(joint_model_group, "init");
  //kinematic_model.setToDefaultValues(kinematic_model.getJointModelGroup("arm"), "init");

  //moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group);


  //planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
 //context->solve(res);


 ros::NodeHandle nh;
 ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1 );
 ros::Rate loop_rate(1);
 for (int cnt=0; cnt<5 && ros::ok(); cnt++)
 {
	 kinematic_state->setToRandomPositions(joint_model_group);

	 moveit_msgs::DisplayRobotState msg;
	 robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
	 robot_state::robotStateToRobotStateMsg(*kinematic_state, robot_state);

	 robot_state_publisher.publish( msg );

	 ros::spinOnce();
	 loop_rate.sleep();
 }
*/

  switch (req.pose)
  {
	case ra1_pro_msgs::MoveFixPose::Request::INIT:
		res.error = ra1_pro_msgs::MoveFixPose::Response::SUCCESS;
		ROS_INFO_STREAM("New fix pose for arm: INIT Pose");
		return true;
	break;

	case ra1_pro_msgs::MoveFixPose::Request::HOME:
		res.error = ra1_pro_msgs::MoveFixPose::Response::SUCCESS;
		ROS_INFO_STREAM("New fix pose for arm: Home Pose");
		return true;
	break;

	case ra1_pro_msgs::MoveFixPose::Request::RANDOM:
	  if (!moveToRandomPose())
	    res.error = res.ERROR;
	  	return true;
    break;

	default:
		res.error = res.ERROR;
		break;
  }

  res.error = res.ERROR;
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ra1_pro_move");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ra1_pro_move::RA1ProMove move_arm_node;
  move_arm_node.init();

  ROS_INFO("Pose move server ready!");
  ros::spin();

  return 0;
}
