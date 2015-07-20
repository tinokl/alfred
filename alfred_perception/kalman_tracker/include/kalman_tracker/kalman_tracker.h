#ifndef KALMAN_TRACKER
#define KALMAN_TRACKER

#include <ros/ros.h>
#include <math.h>
#include <boost/bind.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


namespace kalman_tracker
{

class KalmanTracker
{
  public:

    KalmanTracker();
    virtual ~KalmanTracker();

    void init();

    void getMeasurePose1(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void getMeasurePose2(const geometry_msgs::PoseStamped& msg);
    void getPoseWithCovarianceMeasure(geometry_msgs::PoseWithCovarianceStamped pose_measure);
    void getPoseMeasure(geometry_msgs::PoseStamped pose_measure);

    void publishTransform();

    void resetData();
    void sysCmdCB(const std_msgs::StringConstPtr &msg);

    ros::Subscriber sub_;
    ros::Subscriber sys_cmd_sub_;
    ros::Subscriber measure_1_pose_sub_;
    ros::Subscriber measure_2_pose_sub_;
    ros::Subscriber scan_matcher_pose_sub_;

    ros::Publisher pose_pub_;
    ros::Publisher pose_covar_pub_;

    ros::NodeHandle nh_;

    tf::TransformBroadcaster tf_broadcaster_;

    bool start_pose_;
    geometry_msgs::PoseWithCovarianceStamped robot_pose_;

    ros::Time now_;
    ros::Time prev_;
    cv::KalmanFilter kf_;
    cv::Mat state_;
    cv::Mat measure_;

    int state_size_;
    int meas_size_;
    int contr_size_;

    unsigned int type_;
};

}
#endif
