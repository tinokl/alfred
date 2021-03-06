#include <ros/ros.h>
#include <boost/bind.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <ball_detection/BallDetectionConfig.h>
#include <ra1_pro_msgs/DeltaPoint.h>
#include <std_msgs/Header.h>

namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher div_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub2_;
  dynamic_reconfigure::Server<ball_detection::BallDetectionConfig> config_server_;
  ball_detection::BallDetectionConfig config_;

public:
  ImageConverter()
      : it_(nh_) //, hsv_min_(0, 0, 0), hsv_max_(180, 255, 255)
  {
    image_pub_ = it_.advertise("/ball_detection", 1);
    image_pub2_ = it_.advertise("/image_inbetween", 1);
    image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
    config_server_.setCallback(boost::bind(&ImageConverter::configCallback, this, _1, _2));
    div_pub_ = nh_.advertise<ra1_pro_msgs::DeltaPoint>("/ra1_pro/delta_ptn", 10);
  }

  ~ImageConverter()
  {
  }

  void configCallback(const ball_detection::BallDetectionConfig & config, uint32_t)
  {
      config_ = config;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_inbetween_ptr(new cv_bridge::CvImage());
    cv_inbetween_ptr->encoding = "mono8";

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv_img;
    // Convert it to gray
    cv::cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);

    cv::Scalar hsv_min(config_.min_hue, config_.min_sat, config_.min_val);
    cv::Scalar hsv_max(config_.max_hue, config_.max_sat, config_.max_val);

    cv::Mat bool_img;
    cv::inRange(hsv_img, hsv_min, hsv_max, bool_img);

    // Reduce the noise so we avoid false circle detection
    cv::GaussianBlur(bool_img, bool_img, cv::Size(config_.gauss_size, config_.gauss_size), config_.gauss_sigma);

    cv_inbetween_ptr->image = bool_img;
    image_pub2_.publish(cv_inbetween_ptr->toImageMsg());

    std::vector<cv::Vec3f> circles;

    // Apply the Hough Transform to find the circles
    cv::HoughCircles(bool_img, circles, CV_HOUGH_GRADIENT, 2, bool_img.rows/8, 100, 40, 50, 0 );

    bool detected = false;
    // Draw the circles detected
    cv::Point main_center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        main_center = center;
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 5, 8, 0 );
        detected = true;
     }
    
    if (detected)
    {
		ra1_pro_msgs::DeltaPoint d_msg;
		d_msg.header.frame_id = "/ball";
		d_msg.header.stamp = ros::Time::now();
		d_msg.delta_x = main_center.x - 320;//- 640;
		d_msg.delta_y = main_center.y - 240;//- 360;

		div_pub_.publish(d_msg);
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

