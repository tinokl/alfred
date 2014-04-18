#include <xbox_controller/alfred_teleop_node.h>

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
  ros::NodeHandle private_nh("~");

  double hz;
  private_nh.param<double>("hz", hz, 20.0);

  servo_1_ = 0.0;
  servo_2_ = 0.0;
  servo_3_ = 0.0;
  servo_4_ = 0.0;
  servo_5_ = 0.0;
  servo_6_ = 0.0;

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
      &AlfredTeleopNode::joyCB, this);

  vel_com_pub_ = nh_.advertise<ra1_pro_msgs::Ra1ProVelCmd>("/vel_cmd", 1);

  vel_com_msg_.header.frame_id = "joy_vel_cmd";
  vel_com_msg_.header.seq = 0;

  epsilon_ = 0.01;
  scale_ = 500;
}

void AlfredTeleopNode::sendCmd(unsigned int servo_num)
{
  vel_com_msg_.header.stamp = ros::Time::now();
  vel_com_msg_.header.seq++;
  vel_com_msg_.direction = 0.0;
  vel_com_msg_.position = 0.0;
  vel_com_msg_.command = "Direction"; // Move command direction
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
}

void AlfredTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->buttons[SLEEP])
  {

  }

  if (joy->buttons[POWER])
  {

  }

  if (fabs(joy->axes[TRIGGER_LT] - servo_1_) > epsilon_)
  {
    servo_1_ = joy->axes[TRIGGER_LT];
    sendCmd(1);
  }

  if (fabs(joy->axes[AXIS_UD_RIGHT] - servo_5_) > epsilon_)
  {
    servo_5_ = joy->axes[AXIS_LR_RIGHT];
    sendCmd(5);
  }

  if (fabs(joy->axes[AXIS_LR_RIGHT] - servo_6_) > epsilon_)
  {
    servo_6_ = joy->axes[AXIS_LR_RIGHT];
    sendCmd(6);
  }
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

