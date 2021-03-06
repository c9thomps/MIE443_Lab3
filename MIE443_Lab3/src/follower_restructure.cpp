#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <cmath>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>

cv::Point center = cv::Point(-1,-1);
geometry_msgs::Twist vel;
double neutral;
double depth = 0;

using namespace std;
namespace enc = sensor_msgs::image_encodings;

void depthCallback(const sensor_msgs::ImageConstPtr& dmsg)
{	
	cv::Mat depth_img;

  try{
		depth_img = cv_bridge::toCvShare(dmsg, enc::TYPE_16UC1)->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", dmsg->encoding.c_str());
    depth_img.release();
  }

	if(center.x > 0){
		depth = 0.0;

		depth = double(depth_img.at<short int>(int(center.y), int(center.x)));

		std::cout << depth << std::endl;
	}

	cv::imshow ( "Depth image", depth_img);
	cv::waitKey(1);
	depth_img.release();

}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat rgb_img;
	
  try{
		rgb_img = cv_bridge::toCvShare(msg, "bgr8")->image;
		neutral = rgb_img.cols/2.0;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    rgb_img.release();
  }

	if(!rgb_img.empty()){

		cv::Mat hsv_img, img_print, thresh_img;

		//thresholding/imgproc
		int Hu = 52;
		int Su = 255;
		int Vu = 255;
		int Hl = 19;
		int Sl = 64;
		int Vl = 81;
		int type_blur = 1;
		int blur = 1; 
		int open_type = 0;
		int open_size = 4; 
		int close_size = 2;
		cv::Mat element_open;
		cv::Mat element_close;

		//depth calculation
		double area_max = 0;
		double area = 0;
		int area_index = 0;
		long unsigned int depth;
		cv::Moments mu;
		cv::Point2f mc;
		double center_x = 0;

		cv::Scalar lowerb;
		cv::Scalar upperb;

		std::vector< std::vector< cv::Point> > contours;
		std::vector< cv::Vec4i> hierarchy;

		//convert colors, and blur
		cv::cvtColor( rgb_img, hsv_img, CV_BGR2HSV );
		GaussianBlur( rgb_img, rgb_img, cv::Size( ((blur*2)+1), ((blur*2)+1)), 0, 0 );

		lowerb = cv::Scalar(Hl, Sl, Vl);
		upperb = cv::Scalar(Hu, Su, Vu);
		inRange(hsv_img, lowerb, upperb, thresh_img);

		element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*open_size + 1, 2*open_size+1 ), cv::Point( open_size, open_size ));

		element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*close_size + 1, 2*close_size+1 ), cv::Point( close_size, close_size ));

		//open
		erode(thresh_img,thresh_img,element_open);
		dilate(thresh_img,thresh_img,element_open);

		//close
		dilate(thresh_img,thresh_img,element_close);
		erode(thresh_img,thresh_img,element_close);
				
		//find contours
		findContours(thresh_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		if(contours.size() > 0){
			area_max = 0;
			area = 0;
			area_index = 0;
				
			for(int i = 0; i < contours.size(); ++i){
				area = contourArea(contours[i]);
				if(area > area_max){
					area_max = area;
					area_index = i;
				}
			}
	
			mu = cv::moments( contours[area_index], false );
			mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
	
			cv::circle(rgb_img, mc, 4, cv::Scalar(255,255,255),-1, 8,0);
			//std::cout << "x: " << int(mc.x) << " y: " << int(mc.y) << std::endl;

			center = cv::Point(int(mc.x), int(mc.y));
			cv::circle(rgb_img, mc, 4, cv::Scalar(255,255,255),-1, 8,0);

			//cout << "x: " << center.x << " y: " << center.y << endl;
		} else {
			center = cv::Point(-1,-1);
		}
	}

	cv::imshow ( "rgb_image", rgb_img);
	cv::waitKey(1);
	rgb_img.release();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
	ros::Publisher vel_pub;

	//subscribers
  image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, rgbCallback);
	image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, depthCallback); //--for the kinect
	
	//publishers
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//driving declarations
	vel.angular.z = 0.0;
	vel.linear.x = 0.0;

	double followDist = 1000;
	double linear = 0.0;
	double maxLinear = 1.0;
	
	double angular = 0.0;
	double maxAngular = 0.5;
	
	/*cv::createTrackbar( "Hu", "thresh", &Hu, 255, NULL );
	cv::createTrackbar( "Su", "thresh", &Su, 255, NULL );
	cv::createTrackbar( "Vu", "thresh", &Vu, 255, NULL );
	
	cv::createTrackbar( "Hl", "thresh", &Hl, 255, NULL );
	cv::createTrackbar( "Sl", "thresh", &Sl, 255, NULL );
	cv::createTrackbar( "Vl", "thresh", &Vl, 255, NULL );*/

	//cv::createTrackbar( "Blur", "view", &blur, 10, NULL );
	//cv::createTrackbar( "Type", "view", &type_blur, 2, NULL );
	//cv::createTrackbar( "OpenShape", "view", &open_type, 2, NULL );
	//cv::createTrackbar( "OpenSize", "view", &open_size, 8, NULL );
	//cv::createTrackbar( "CloseSize", "view", &close_size, 8, NULL );
	//cv::createTrackbar( "distance", "rgb_img", &curr_dist, 200, NULL);
  
	while(ros::ok()){
		
		vel.angular.z = 0.0;
    vel.linear.x = 0.0;

		if(depth > 0){
			//linear
			linear = ((depth - followDist)/followDist)*maxLinear;
			if(abs(linear) > maxLinear){
				linear = copysignf(maxLinear, linear);
			}

			//angular
			angular = ((neutral - center.x)/neutral)*maxAngular;
			if(abs(angular) > maxAngular){
				angular = copysignf(maxAngular, angular);
			}

			cout << "linear: " << linear << endl;
			cout << "angular: " << angular << endl;

			vel.angular.z = angular;
    	vel.linear.x = linear;
		}

		vel_pub.publish(vel);
 
  	ros::spinOnce();
  }

	return 0;
}
