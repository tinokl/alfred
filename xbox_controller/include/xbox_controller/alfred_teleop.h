#ifndef alfred_teleop_h___
#define alfred_teleop_h___

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

namespace AlfredTeleop
{

class AlfredTeleopNode
{
protected:
    enum JoyButton {
      JOY_BUTTON_SLEEP = 11,
    };

    ros::NodeHandle nh_;

    ros::Timer cmd_generator_timer_;

    ros::Publisher sh_pitch_pub_;
    ros::Publisher sh_yaw_pub_;

    ros::Subscriber joy_sub_;

    double std_trans_vel_;
    double std_rot_vel_;

    std_msgs::Float64 sh_pitch_;
    std_msgs::Float64 sh_yaw_;

    double std_delta_pitch_;
    double std_delta_yaw_;

    double delta_pitch_;
    double delta_yaw_;

    double sh_pitch_max_;
    double sh_pitch_min_;
    double sh_yaw_max_;
    double sh_yaw_min_;

    bool repeat_commands_;

public:
    AlfredTeleopNode();

    virtual ~AlfredTeleopNode();

    void init();

protected:
    void cmdGeneratorTimerCB(const ros::TimerEvent& e);

    void joyCB(const sensor_msgs::Joy::ConstPtr& joy);

    void sendCmd();

    static double limit(double value, double min, double max);
};

} // end namespace AlfredTeleop

#endif // alfred_teleop_h___
