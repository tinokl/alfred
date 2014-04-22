#ifndef alfred_teleop_h___
#define alfred_teleop_h___

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <ra1_pro_msgs/Ra1ProVelCmd.h>

namespace AlfredTeleop
{

class AlfredTeleopNode
{
protected:
    enum JoyButton {
      POWER = 7,
      SLEEP = 6,
      AXIS_LR_LEFT = 0,
      AXIS_UD_LEFT = 1,
      AXIS_LR_RIGHT = 2,
      AXIS_UD_RIGHT = 3,
      TRIGGER_RT = 4,
      TRIGGER_LT = 5
    };

    ros::NodeHandle nh_;

    ros::Subscriber joy_sub_;

    ros::Publisher vel_com_pub_;

    ra1_pro_msgs::Ra1ProVelCmd vel_com_msg_;

    double servo_1_;
    double servo_2_;
    double servo_3_;
    double servo_4_;
    double servo_5_;
    double servo_6_;

    double epsilon_;
    double scale_;

 /*
    double std_trans_vel_;
    double std_rot_vel_;

    double std_delta_pitch_;
    double std_delta_yaw_;

    double delta_pitch_;
    double delta_yaw_;

    double sh_pitch_max_;
    double sh_pitch_min_;
    double sh_yaw_max_;
    double sh_yaw_min_;
*/
    bool commands_s1;
    bool commands_s2;
    bool commands_s3;
    bool commands_s4;
    bool commands_s5;
    bool commands_s6;

public:
    AlfredTeleopNode();

    virtual ~AlfredTeleopNode();

    void init();

protected:

    void joyCB(const sensor_msgs::Joy::ConstPtr& joy);

    void sendCmd(unsigned int);

    static double limit(double value, double min, double max);
};

} // end namespace AlfredTeleop

#endif // alfred_teleop_h___
