#ifndef ALFRED_OCCIPITAL_LOBE
#define ALFRED_OCCIPITAL_LOBE

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
#include <stdio.h>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";

namespace alfred_occipital_lobe
{

    class AlfredOccipitalLobe
    {
    public:
        AlfredOccipitalLobe();
        virtual ~AlfredOccipitalLobe();
        void init();

    private:

        AlfredOccipitalLobe(const AlfredOccipitalLobe &src);

        //int cam_idx_;
        //cv::VideoCapture cap_;

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        //image_transport::Publisher image_pub_;
        ros::Subscriber camera_sub_;

        void detection(const sensor_msgs::ImageConstPtr& msg);
        void classification(cv::Mat input);
        void scaleDownImage(cv::Mat &originalImg,cv::Mat &scaledDownImage);
        void convertToPixelValueArray(cv::Mat &img,int pixelarray[]);
        void cropImage(cv::Mat &originalImage,cv::Mat &croppedImage);

        std::string topic_;
        //std::string target_frame;
        double fov_x;
        double fov_y;

        bool debug_;
        bool train_;
        //TransformationWithRaycasting *transformation_;

    };

}
#endif
