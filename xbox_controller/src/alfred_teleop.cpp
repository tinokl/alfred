#include <xbox_controller/alfred_teleop.h>

namespace AlfredTeleop
{

AlfredTeleopNode::AlfredTeleopNode()
    : repeat_commands_(false)
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

  private_nh.param<double>("sh_pitch_max", sh_pitch_max_, 0.76);
  private_nh.param<double>("sh_pitch_min", sh_pitch_min_, -0.82);
  private_nh.param<double>("sh_yaw_max", sh_yaw_max_, 2.58);
  private_nh.param<double>("sh_yaw_min", sh_yaw_min_, -2.58);

  private_nh.param<double>("std_trans_vel", std_trans_vel_, 0.2);
  private_nh.param<double>("std_rot_vel", std_rot_vel_, 1.0);
  private_nh.param<double>("std_delta_pitch", std_delta_pitch_, 0.02);
  private_nh.param<double>("std_delta_yaw", std_delta_yaw_, 0.02);

  sh_pitch_.data = 0.0;
  sh_yaw_.data = 0.0;

  cmd_generator_timer_ = nh_.createTimer(ros::Duration(1.0 / hz),
      &AlfredTeleopNode::cmdGeneratorTimerCB, this, false);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
      &AlfredTeleopNode::joyCB, this);

  sh_pitch_pub_ = nh_.advertise<std_msgs::Float64>(
      "/sh_pitch_controller/command", 1);
  sh_yaw_pub_ = nh_.advertise<std_msgs::Float64>("/sh_yaw_controller/command",
      1);
}

void AlfredTeleopNode::cmdGeneratorTimerCB(const ros::TimerEvent& e)
{
  if (repeat_commands_)
    sendCmd();
}

void AlfredTeleopNode::sendCmd()
{
  sh_pitch_.data = limit(sh_pitch_.data + delta_pitch_, sh_pitch_min_,
      sh_pitch_max_);
  sh_yaw_.data = limit(sh_yaw_.data + delta_yaw_, sh_yaw_min_, sh_yaw_max_);

  sh_pitch_pub_.publish(sh_pitch_);
  sh_yaw_pub_.publish(sh_yaw_);
}

void AlfredTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr& joy)
{
  bool have_commands = false;

 // if (joy->buttons[JOY_BUTTON_SENSOR_HEAD_DEADMAN_SWITCH])
 // {
    have_commands = true;
    if (joy->buttons[JOY_BUTTON_SLEEP])
    {
      sh_pitch_.data = 0.0;
      sh_yaw_.data = 0.0;
      delta_pitch_ = 0;
      delta_yaw_ = 0;
    }
    else
    {
      delta_pitch_ = std_delta_pitch_ * joy->axes[2];
      delta_yaw_ = std_delta_yaw_ * joy->axes[3];
    }
 // }
  //else
  //{
  //  delta_pitch_ = 0;
  //  delta_yaw_ = 0;
  //}

  // Send commands if we have some; else, send "stop" commands at least once:
  if (have_commands || repeat_commands_)
  {
    sendCmd();
    repeat_commands_ = have_commands;
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

