#include <alfred_joy/alfred_teleop_node.h>

namespace AlfredTeleop
{

AlfredTeleopNode::AlfredTeleopNode()
{
}

AlfredTeleopNode::~AlfredTeleopNode()
{
}

void AlfredTeleopNode::init()
{
  //ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  double hz;
  nh.param<double>("hz", hz, 20.0);

  servo_1_ = 0.0;
  servo_2_ = 0.0;
  servo_3_ = 0.0;
  servo_4_ = 0.0;
  servo_5_ = 0.0;
  servo_6_ = 0.0;

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &AlfredTeleopNode::joyCB, this);

  //vel_com_pub_ = nh_.advertise<ra1_pro_msgs::Ra1ProVelCmd>("/vel_cmd", 1);
  move_pose_client_ = nh.serviceClient<ra1_pro_msgs::MovePose>("move_pose");

  //vel_com_msg_.header.frame_id = "joy_vel_cmd";
  //vel_com_msg_.header.seq = 0;

  epsilon_ = 0.3;
  scale_ = 10;
}

int AlfredTeleopNode::mapGripper(float input)
{
  // gamepad: -1 to +1 => -800 to -100
  input = input * (-1.0);
  if (input > 0.75)
    input = 1;
  if (input < -0.75)
    input = -1;
  // range new from 1 to +3
  input = input + 2;
  //X between A and B, Y to fall between C and D
  //Y = (X-A)/(B-A) * (D-C) + C
  input = (input - 1.0)/(3.0 - 1.0) * (900.0 - 10.0) + 10.0;
  // 100 to 800 and negate it later
  return -input;
}

void AlfredTeleopNode::sendPosCmd(unsigned int servo_num, float input)
{
	/*
  vel_com_msg_.header.stamp = ros::Time::now();
  vel_com_msg_.header.seq++;
  vel_com_msg_.direction = 0.0;
  if (servo_num == 1)
    vel_com_msg_.position = mapGripper(input);
  else
    vel_com_msg_.position = (int)input;
  vel_com_msg_.command = "POS"; // Move command position
  vel_com_msg_.servo = servo_num;

  vel_com_pub_.publish( vel_com_msg_ );
  */
}

void AlfredTeleopNode::sendDirCmd(unsigned int servo_num)
{
	/*
  vel_com_msg_.header.stamp = ros::Time::now();
  vel_com_msg_.header.seq++;
  vel_com_msg_.direction = 0.0;
  vel_com_msg_.position = 0.0;
  vel_com_msg_.command = "DIR"; // Move command direction
  vel_com_msg_.servo = servo_num;

  switch(servo_num)
  {
    case 1: vel_com_msg_.direction = (double)((int)(scale_ * servo_1_)); break;
    case 2: vel_com_msg_.direction = (double)((int)(scale_ * servo_2_)); break;
    case 3: vel_com_msg_.direction = (double)((int)(scale_ * servo_3_)); break;
    case 4: vel_com_msg_.direction = (double)((int)(scale_ * servo_4_)); break;
    case 5: vel_com_msg_.direction = (double)((int)(scale_ * servo_5_)); break;
    case 6: vel_com_msg_.direction = (double)((int)(scale_ * servo_6_)); break;
    default: ROS_ERROR("Alfred Teleop Node: Wrong Servo Number!");
  }

  vel_com_pub_.publish( vel_com_msg_ );
  */
}

void AlfredTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr& joy)
{
/*
  if (joy->buttons[SLEEP])
  {

  }

  if (joy->buttons[POWER])
  {
    vel_com_msg_.header.stamp = ros::Time::now();
    vel_com_msg_.header.seq++;
    vel_com_msg_.direction = 0.0;
    vel_com_msg_.position = 0.0;
    vel_com_msg_.command = "START";
    vel_com_msg_.servo = 0.0;
    vel_com_pub_.publish( vel_com_msg_ );
  }

  if (fabs(joy->axes[TRIGGER_LT] - servo_1_) > epsilon_)
  {
    ROS_ERROR_STREAM("This is the true value: " << joy->axes[TRIGGER_LT]);
    servo_1_ = joy->axes[TRIGGER_LT];
    sendPosCmd(1,joy->axes[TRIGGER_LT]);
  }

  */

  if (fabs(joy->axes[AXIS_UD_RIGHT] - servo_5_) > epsilon_)
  {
    servo_5_ = joy->axes[AXIS_LR_RIGHT];
    //sendDirCmd(5);
  }

  if (fabs(joy->axes[AXIS_LR_RIGHT] - servo_6_) > epsilon_)
  {
    servo_6_ = joy->axes[AXIS_LR_RIGHT];
    //sendDirCmd(6);
  }

/*
  ra1_pro_msgs::MovePose srv;
  if (move_pose_client_.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)move_pose_client_.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    //return 1;
  }
*/
}

double AlfredTeleopNode::limit(double value, double min, double max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

} // end namespace AlfredTeleop

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "AlfredTeleop");
    AlfredTeleop::AlfredTeleopNode Alfred_teleop_node;

    Alfred_teleop_node.init();

    ros::spin();
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}

