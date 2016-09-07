#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
		img = cv_bridge::toCvShare(msg, "16UC1")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    img.release();
  }
}

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/depth_registered/image_raw", 1, imageCallback);
  //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

	cv::Mat img_print, img_scene;

  while(ros::ok()){
    if(!img.empty()){
			img.copyTo(img_print);
			img.release();

			std::cout << img_print.at<int>(240, 240) << std::endl;
			cv::circle(img_print, cv::Point(240,240), 4, cv::Scalar(255,255,255),-1, 8,0);
			cout << "here" << endl;

			cv::imshow("view", img_print);
			cv::waitKey(5);
    }
  	ros::spinOnce();
  }
  cv::destroyWindow("view");
}
