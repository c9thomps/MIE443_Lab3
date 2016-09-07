#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <cmath>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>

cv::Mat global_rgb, global_depth;
using namespace std;

void depthCallback(const sensor_msgs::ImageConstPtr& dmsg)
{	
  try{
		global_depth = cv_bridge::toCvShare(dmsg)->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", dmsg->encoding.c_str());
    global_depth.release();
  }
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
		global_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
		//cv::imshow("view", rgb_img);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    global_rgb.release();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("thresh");
	//cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
	ros::Publisher vel_pub;

	//subscribers
  image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, rgbCallback);
	image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, depthCallback); //--for the kinect
	
	//publishers
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	cv::Mat depth_img, rgb_img, hsv_img, img_print, thresh_img;
	
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

	//driving
	double linear = 0.5;
	double angular = 0.0;
	geometry_msgs::Twist vel;
	double follow_dist = 1.00;
	int curr_dist = 100;
	double neutral = 0;
	double maxLinear = 1.00;
	double maxAngular = 0.5;

	cv::Scalar lowerb;
	cv::Scalar upperb;

	std::vector< std::vector< cv::Point> > contours;
	std::vector< cv::Vec4i> hierarchy;
	
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
	cv::createTrackbar( "distance", "view", &curr_dist, 200, NULL);
  
	while(ros::ok()){
		center_x =-1;

    if(!global_rgb.empty()){
			if(!global_depth.empty()){
				global_depth.copyTo(depth_img);
				//depth_img = global_depth;
				global_rgb.copyTo(rgb_img);
				//rgb_img = global_rgb;
				
				global_rgb.release();
				global_depth.release();

				cv::cvtColor( rgb_img, hsv_img, CV_BGR2HSV );
				
				if(type_blur == 2){
					medianBlur(rgb_img, rgb_img, ((blur*2)+1));
				}
				if(type_blur == 1){
					 GaussianBlur( rgb_img, rgb_img, cv::Size( ((blur*2)+1), ((blur*2)+1)), 0, 0 );
				}

				lowerb = cv::Scalar(Hl, Sl, Vl);
				upperb = cv::Scalar(Hu, Su, Vu);
				inRange(hsv_img, lowerb, upperb, thresh_img);

				element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*open_size + 1, 2*open_size+1 ), cv::Point( open_size, open_size ));

				element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*close_size + 1, 2*close_size+1 ), cv::Point( close_size, close_size ));

				erode(thresh_img,thresh_img,element_open);
				dilate(thresh_img,thresh_img,element_open);

				dilate(thresh_img,thresh_img,element_close);
				erode(thresh_img,thresh_img,element_close);
				
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

					std::cout << "here1" << std::endl;

					mu = cv::moments( contours[area_index], false );
					mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

					cv::circle(depth_img, mc, 4, cv::Scalar(255,255,255),-1, 8,0);
					std::cout << "x: " << int(mc.x) << " y: " << int(mc.y) << std::endl;
					std::cout << depth_img.at<int>(int(mc.x), int(mc.y)) << std::endl;
					center_x = mc.x;
				}

				 /// Get the moments
  			/*std::vector<cv::Moments> mu(contours.size() );
  			for( int i = 0; i < contours.size(); i++ ){
					mu[i] = cv::moments( contours[i], false );
				}

  			///  Get the mass centers:
  			std::vector<cv::Point2f> mc( contours.size() );
 				for( int i = 0; i < contours.size(); i++ ){ 
					mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				}

				for(int i = 0; i < mc.size(); i++){
					cv::circle(rgb_img, mc[i], 4, cv::Scalar(0,0,0),-1, 8,0);
				}*/
			}
    }
		
		if(!thresh_img.empty()){
			cv::imshow("thresh", depth_img);
			cv::imshow("view", rgb_img);
			cv::waitKey(5);
		}

		//Driving
		angular = 0.0;
		linear = 0.0;

		if(center_x > 0){
			//linear
			linear = (double(curr_dist/100.00) - follow_dist)*maxLinear;
			if(abs(linear) > maxLinear){
				linear = copysignf(maxLinear, linear);
			}

			//angular
			neutral = double(rgb_img.cols/2.0);
			angular = ((neutral - center_x)/neutral)*maxAngular;
			if(abs(angular) > maxAngular){
				angular = copysignf(maxAngular, angular);
			}
		}

		//std::cout << "angular speed: " << angular << std::endl;

    vel.angular.z = angular;
    vel.linear.x = linear;
 
    vel_pub.publish(vel);
 
  	ros::spinOnce();
  }
  //cv::destroyWindow("thresh");
	//cv::destroyWindow("view");
}
