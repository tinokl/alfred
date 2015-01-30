#include <alfred_haarcascade/alfred_haarcascade_node.h>

static const std::string OPENCV_WINDOW = "Image window";

namespace tedusar_haarcascade_detection
{

TedusarHaarcascadeDetection::TedusarHaarcascadeDetection()
: it_(nh_)
{
	cv::namedWindow(OPENCV_WINDOW);
}

TedusarHaarcascadeDetection::~TedusarHaarcascadeDetection()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void TedusarHaarcascadeDetection::init()
{
    
    ros::NodeHandle nh("~");

    nh.getParam("subscribe_cam", sub_cam_);
    nh.getParam("haarcascade_file", haar_file_);

    nh.getParam("debug", debug_);
    nh.getParam("biggest", biggest_);

    std::string file = boost::lexical_cast<std::string>(haar_file_);
    classifier_ = cv::CascadeClassifier(file);

    image_sub_ = it_.subscribe(sub_cam_, 1, &TedusarHaarcascadeDetection::detector, this);
    div_pub_ = nh_.advertise<ra1_pro_msgs::DeltaPoint>("/ra1_pro/delta_ptn", 10);
}

void TedusarHaarcascadeDetection::detector(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img = cv_ptr->image;

  std::vector<cv::Rect> detections;
  cv::Mat img_gray;

  cv::cvtColor( img, img_gray, CV_BGR2GRAY );
  cv::equalizeHist( img_gray, img_gray );

  //-- Detect
  //classifier_.detectMultiScale( img_gray, detections, 1.05, 6, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30), cv::Size(200,200));
    classifier_.detectMultiScale( img_gray, detections, 1.05, 8 );

  /*
  :detectMultiScale 	( const Mat & image,
		CV_OUT vector< Rect > &  	objects,
		double  	scaleFactor = 1.1,
		1.05 is a good possible value for this, which means you use a small step for resizing

		int  	minNeighbors = 3,
		This parameter will affect the quality of the detected faces.
		Higher value results in less detections but with higher quality.
		3~6 is a good value for it.

		int  	flags = 0,
		Size  	minSize = Size(),
		Size  	maxSize = Size()
  */

  if (biggest_)
  {
	  bool detected = false;
	  cv::Rect biggest_rect(0,0,0,0);
	  for(size_t i = 0; i < detections.size(); ++i)
	  {
	     cv::Rect rect = detections[i];
	     if(rect.width > biggest_rect.width)
	    	 biggest_rect = rect;
	     detected = true;
	  }

	  cv::Point p1( biggest_rect.x,biggest_rect.y );
	  cv::Point p2( biggest_rect.x + biggest_rect.width, biggest_rect.y + biggest_rect.height );

	  cv::Point center( biggest_rect.x + biggest_rect.width/2, biggest_rect.y + biggest_rect.height/2);

	  cv::rectangle(img, p1, p2, cv::Scalar( 255, 0 ,0 ), 4, 8, 0 );

	  if (detected)
	  {
		  ra1_pro_msgs::DeltaPoint d_msg;
		  d_msg.header.frame_id = "/face";
		  d_msg.header.stamp = ros::Time::now();
		  d_msg.delta_x = center.x - 320;//- 640;
		  d_msg.delta_y = center.y - 240;//- 360;
		  div_pub_.publish(d_msg);
	  }
  }
  else
  {
	  for( size_t i = 0; i < detections.size(); i++ )
	  {
		//cv::Point center( detections[i].x + detections[i].width*0.5, detections[i].y + detections[i].height*0.5 );
		//cv::ellipse( img, center, cv::Size( detections[i].width*0.5, detections[i].height*0.5), 0, 0, 360, cv::Scalar( 0, 255, 0 ), 4, 8, 0 );
		cv::Point p1( detections[i].x,detections[i].y );
		cv::Point p2( detections[i].x + detections[i].width, detections[i].y + detections[i].height );
		cv::rectangle(img, p1, p2, cv::Scalar( 255, 0 ,0 ), 4, 8, 0 );
	  }
  }

  if (debug_)
  {
	  // Update GUI Window
	  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	  cv::waitKey(3);
  }
}

}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "tedusar_haarcascade_detection" );
        tedusar_haarcascade_detection::TedusarHaarcascadeDetection tedusar_haarcascade_detection;
        tedusar_haarcascade_detection.init();
        ros::spin();
    }
    catch( ... )
    {
        ROS_ERROR_NAMED("tedusar_haarcascade_detection","Unhandled exception!");
        return -1;
    }

    return 0;
}
