#include <kalman_tracker/kalman_tracker.h>

namespace kalman_tracker
{

KalmanTrackerNode::KalmanTrackerNode()
{
  start_pose_ = false;
  state_size_ = 8; // [x, y, z, v_x, v_y, v_z, theta, v_theta]
  meas_size_ = 4; // [x, y, z, theta]
  contr_size_ = 0;
  type_ = CV_32F;
}

KalmanTrackerNode::~KalmanTrackerNode()
{

}

void KalmanTrackerNode::init()
{
    ros::NodeHandle nh("~");

    sys_cmd_sub_ = nh.subscribe("/syscommand", 1, &KalmanTrackerNode::sysCmdCB, this);
//    measure_1_pose_sub_ = nh_.subscribe("/elevation_pose", 1, &KalmanTrackerNode::getMeasurePose1, this);
//    measure_2_pose_sub_ = nh_.subscribe("/estimated_pose", 1, &KalmanTrackerNode::getMeasurePose2, this);
    measure_1_pose_sub_ = nh_.subscribe("/poseupdate", 1, &KalmanTrackerNode::getMeasurePose1, this);
    measure_2_pose_sub_ = nh_.subscribe("/estimated_pose", 1, &KalmanTrackerNode::getMeasurePose2, this);

    pose_covar_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose_simple", 1);

    ros::Rate loop_rate(20);

    resetData();

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1  0  0 dT  0  0  0  0 ] dT => 3
    // [ 0  1  0  0 dT  0  0  0 ] dT => 12
    // [ 0  0  1  0  0 dT  0  0 ] dT => 21
    // [ 0  0  0  1  0  0  0  0 ]
    // [ 0  0  0  0  1  0  0  0 ]
    // [ 0  0  0  0  0  1  0  0 ]
    // [ 0  0  0  0  0  0  1  dT] dT => 55
    // [ 0  0  0  0  0  0  0  1 ]
    cv::setIdentity(kf_.transitionMatrix);

    // Measure Matrix H
    //   x y z vx vy vz theta vtheta
    // [ 1 0 0 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 0 0 ]
    // [ 0 0 1 0 0 0 0 0 ]
    // [ 0 0 0 0 0 0 1 0 ]
    kf_.measurementMatrix = cv::Mat::zeros(meas_size_, state_size_, type_);
    kf_.measurementMatrix.at<float>(0) = 1.0f;
    kf_.measurementMatrix.at<float>(9) = 1.0f;
    kf_.measurementMatrix.at<float>(18) = 1.0f;
    kf_.measurementMatrix.at<float>(30) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex 0  0  0    0    0    0      0        ]
    // [ 0  Ey 0  0    0    0    0      0        ]
    // [ 0  0  Ez 0    0    0    0      0        ]
    // [ 0  0  0  Ev_x 0    0    0      0        ]
    // [ 0  0  0  0    Ev_y 0    0      0        ]
    // [ 0  0  0  0    0    Ev_z 0      0        ]
    // [ 0  0  0  0    0    0    Etheta 0        ]
    // [ 0  0  0  0    0    0    0      Ev_theta ]
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-2));
//    kf_.processNoiseCov.at<float>(0)  = 1e-2;
//    kf_.processNoiseCov.at<float>(7)  = 1e-2;
//    kf_.processNoiseCov.at<float>(14) = 1e-2;
    kf_.processNoiseCov.at<float>(27) = 5.0f; // Ev_x
    kf_.processNoiseCov.at<float>(36) = 5.0f; // Ev_y
    kf_.processNoiseCov.at<float>(45) = 5.0f; // Ev_z
    kf_.processNoiseCov.at<float>(54) = 5.0f; // Ev_theta

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(1e-1));

    ROS_INFO("[KalmanTrackerNode::init] Pose estimation node is ready!");

    now_ = ros::Time::now();
    while (ros::ok())
    {
      prev_ = now_;
      now_ = ros::Time::now();

      double dt = ros::Time(0).toSec();
      if (prev_.isValid())
        dt = (now_ - prev_).toSec();

      if (start_pose_)
      {
        kf_.transitionMatrix.at<float>(3) = dt;
        kf_.transitionMatrix.at<float>(12) = dt;
        kf_.transitionMatrix.at<float>(21) = dt;
        kf_.transitionMatrix.at<float>(55) = dt;

        // let the kalman magic happen :P
        state_ = kf_.predict();

        robot_pose_.pose.pose.position.x = state_.at<float>(0);
        robot_pose_.pose.pose.position.y = state_.at<float>(1);
        robot_pose_.pose.pose.position.z = state_.at<float>(2);


        tf::Quaternion quaternion;
        quaternion.setEuler(0.0, 0.0, state_.at<float>(6));
        tf::Transform orientation(quaternion, tf::Vector3(0,0,0));

        robot_pose_.pose.pose.orientation.x = quaternion.getX();
        robot_pose_.pose.pose.orientation.y = quaternion.getY();
        robot_pose_.pose.pose.orientation.z = quaternion.getZ();
        robot_pose_.pose.pose.orientation.w = quaternion.getW();
      }

      robot_pose_.header.stamp = now_;
      robot_pose_.header.frame_id = "map";

      geometry_msgs::PoseStamped robot_pose_simple = geometry_msgs::PoseStamped();
      robot_pose_simple.header = robot_pose_.header;
      robot_pose_simple.pose.position = robot_pose_.pose.pose.position;
      robot_pose_simple.pose.orientation = robot_pose_.pose.pose.orientation;

      pose_covar_pub_.publish(robot_pose_);
      pose_pub_.publish(robot_pose_simple);
      publishTransform();

      ros::spinOnce();
      loop_rate.sleep();
    }
}

void KalmanTrackerNode::sysCmdCB(const std_msgs::StringConstPtr &msg)
{
    if(msg->data == "reset")
    {
      resetData();
    }
}

void KalmanTrackerNode::resetData()
{
  robot_pose_ = geometry_msgs::PoseWithCovarianceStamped();

  kf_ = cv::KalmanFilter(state_size_, meas_size_, contr_size_, type_);
  state_ = cv::Mat(state_size_, 1, type_);
  measure_ = cv::Mat(meas_size_, 1, type_);
  //cv::Mat procNoise(state_size, 1, type)
  // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
}

void KalmanTrackerNode::getMeasurePose1(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  getPoseWithCovarianceMeasure(msg);
}

void KalmanTrackerNode::getMeasurePose2(const geometry_msgs::PoseStamped& msg)
{
  getPoseMeasure(msg);
}

void KalmanTrackerNode::getPoseWithCovarianceMeasure(geometry_msgs::PoseWithCovarianceStamped pose_measure)
{
  geometry_msgs::PoseStamped pose_stamped = geometry_msgs::PoseStamped();
  pose_stamped.pose.position = pose_measure.pose.pose.position;
  pose_stamped.pose.orientation = pose_measure.pose.pose.orientation;

  getPoseMeasure(pose_stamped);
}

void KalmanTrackerNode::getPoseMeasure(geometry_msgs::PoseStamped pose_measure)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(pose_measure.pose.orientation, quaternion);
  tf::Transform orientation(quaternion, tf::Vector3(0,0,0));

  double roll, pitch, yaw;
  orientation.getBasis().getEulerYPR(yaw, pitch, roll);

  measure_.at<float>(0) = pose_measure.pose.position.x;
  measure_.at<float>(1) = pose_measure.pose.position.y;
  measure_.at<float>(2) = pose_measure.pose.position.z;
  measure_.at<float>(3) = yaw;

  //ROS_ERROR_STREAM("measurement x: " << pose_measure.pose.pose.position.x << " y: " << pose_measure.pose.pose.position.y << " z: " << pose_measure.pose.pose.position.z);

  if (start_pose_)
    kf_.correct(measure_);
  else
  {
    kf_.errorCovPre.at<float>(0) = 1;
    kf_.errorCovPre.at<float>(7) = 1;
    kf_.errorCovPre.at<float>(14) = 1;
    kf_.errorCovPre.at<float>(21) = 1;
    kf_.errorCovPre.at<float>(28) = 1;
    kf_.errorCovPre.at<float>(35) = 1;

    state_.at<float>(0) = measure_.at<float>(0);
    state_.at<float>(1) = measure_.at<float>(1);
    state_.at<float>(2) = measure_.at<float>(2);
    state_.at<float>(3) = 0.0;
    state_.at<float>(4) = 0.0;
    state_.at<float>(5) = 0.0;

    kf_.statePost = state_;

    start_pose_ = true;
  }
}

void KalmanTrackerNode::publishTransform()
{
  tf::Quaternion quaternion;
  tf::Vector3 position(state_.at<float>(0), state_.at<float>(1), state_.at<float>(2));

  quaternion.setEuler(0.0, 0.0, state_.at<float>(6));
  tf::Transform orientation(quaternion, tf::Vector3(0,0,0));

  tf::StampedTransform transformation(
               tf::Transform(quaternion, position),
               ros::Time::now(),
               "map",
               "pose_estimation");

   tf_broadcaster_.sendTransform(transformation);
}

}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "kalman_tracker_node");
        kalman_tracker::KalmanTrackerNode pose_estimation_node;
        pose_estimation_node.init();

        while (ros::ok())
            ros::spin();
    }
    catch (...)
    {
        ROS_ERROR_NAMED("kalman_tracker_node", "Unhandled exception!");
        return -1;
    }

    return 0;
}
