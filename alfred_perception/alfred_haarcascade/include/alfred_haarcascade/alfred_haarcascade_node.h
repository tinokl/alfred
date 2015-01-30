#ifndef TEDUSAR_HAARCASCADE_DETECTION
#define TEDUSAR_HAARCASCADE_DETECTION

#include <iostream>
#include <iomanip>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <ra1_pro_msgs/DeltaPoint.h>
#include <stdio.h>
#include <fstream>

namespace tedusar_haarcascade_detection
{
    class TedusarHaarcascadeDetection
    {

        public:

            TedusarHaarcascadeDetection();
            virtual ~TedusarHaarcascadeDetection();
            void init();

        private:

            TedusarHaarcascadeDetection(const TedusarHaarcascadeDetection &src);

            std::string sub_cam_;
            std::string haar_file_;

            bool debug_;
            bool biggest_;

            cv::CascadeClassifier classifier_;

            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;

            ros::Subscriber camera_sub_;
            ros::Publisher div_pub_;

            void detector(const sensor_msgs::ImageConstPtr& msg);
    };

}
#endif
