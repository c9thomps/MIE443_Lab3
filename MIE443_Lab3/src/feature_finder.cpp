#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <memory.h>
#include <math.h> 
#include <cv.h>
#include <cv_bridge/cv_bridge.h>

//-------------------------global-----------------------
cv::Mat img_color, img;

//------------------------callback----------------------
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
		cv::Mat temp;
		temp = cv_bridge::toCvShare(msg, "bgr8")->image;
		temp.copyTo(img_color);
    cv::cvtColor(temp, img, 6);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    img.release();
  }
}

//---------------------------main--------------------------
int main(int argc, char **argv)
{
	cv::initModule_nonfree();
	
	//output stuff
	std::ofstream file;
	std::stringstream ss;
	std::string temp;
	char* path = "/home/turtlebot/Documents/Surf_Testing/";
	char* append = ".csv";
	char* arg;
	char* foundChar;
	char* timeStamp;
	char* writePath;
	bool output;
	bool found;
	int size;
	double startSecs;
	double currSecs;

//ros stuff
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("camera/image/", 1, imageCallback); //--for the webcam
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback); //--for the kinect

	double areaTransform, areaBounding;
	int foundThresh = 60;
	int areaNorm;
	cv::Rect bounding;

	std::vector<cv::Scalar> color;
	color.push_back(cv::Scalar(255, 0, 0));
	color.push_back(cv::Scalar(0, 255, 0));
	color.push_back(cv::Scalar(0, 0, 255));
	color.push_back(cv::Scalar(255, 255, 255));

	std::vector<cv::Mat> imgs_track;
	
	//output results to file
	if( argc == 2 ){ 
		arg = argv[1];

		ss.str("");
		ss << path << arg << append;
		temp = ss.str();
		writePath = (char*)temp.c_str();		

		file.open(writePath);
		
		if(file.is_open()){
			output = true;
			startSecs = ros::Time::now().toSec();
		} else {
			output = false;
			ROS_INFO("Could not open file, check name");
		}
	} else {
		output = false;
	}

	//load images if you want to add more or remove images just either add another line of comment one out, **NO OTHER CHANGES NEED TO BE MADE**
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/cereal_tracking1.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/cereal_tracking2.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/cereal_tracking3.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	
	//check if image loading worked
	for(int i = 0; i < imgs_track.size(); i++){
		if(!imgs_track[i].data){
			return -1;
		}
	}

	cv::Mat img_scene, img_grey, img_print, img_matches, H;

	//image processing for loaded images - keypoints
	int minHessian = 400;
  cv::SurfFeatureDetector detector( minHessian );

  std::vector<std::vector<cv::KeyPoint> > keypoints_track; 
	std::vector<cv::KeyPoint> keypoints_scene;
	std::vector<cv::KeyPoint> keypoints;

	for(int i = 0; i < imgs_track.size(); i++){
  	detector.detect( imgs_track[i], keypoints);
		keypoints_track.push_back(keypoints);
		keypoints.clear();
	}

	//image processing stuff for loaded images - detector
	cv::SurfDescriptorExtractor extractor;

  std::vector<cv::Mat> descriptors_track;
	cv::Mat descriptors_scene;
	cv::Mat descriptors;

	for(int i = 0; i < imgs_track.size(); i++){
  	extractor.compute( imgs_track[i], keypoints_track[i], descriptors);
		descriptors_track.push_back(descriptors);
		descriptors.release(); 
		if(descriptors_track[i].empty()){
			ROS_INFO("this is empty");
		}
	}
	
	//initializing matcher
  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches, good_matches;
  double max_dist = 0; double min_dist = 100;

  std::vector<cv::Point2f> track;
  std::vector<cv::Point2f> scene;

	std::vector<cv::Point2f> track_corners(4);
	std::vector<cv::Point2f> scene_corners(4);

	double time;

	//Video while loop
  while(1){
		time = ros::Time::now().toSec();

    if(!img.empty()){
			img_scene.release();
			img_print.release();
			img_scene = img;
			img_print = img_color;

			img.release();
			img_color.release();
			
			//reinitialize output
			foundChar = "";

			//detect keypoints from video frame
			detector.detect( img_scene, keypoints_scene);
			
			if(keypoints_scene.size() > 0){
				//detector for video frame
		  	extractor.compute( img_scene, keypoints_scene, descriptors_scene );

        //main 'for' loop - compare each loaded picture to video frame
				for(int i = 0; i < imgs_track.size(); i++){
					found = false;
					good_matches.clear();
					matches.clear();

					//get matches for current loaded picture
					matcher.match( descriptors_track[i], descriptors_scene, matches );

					//find valid matches
					for( int j = 0; j < descriptors_track[j].rows; j++ ){ 
						double dist = matches[j].distance;
  	  			if( dist < min_dist ) min_dist = dist;
  	 				if( dist > max_dist ) max_dist = dist;
  				}

					for( int j = 0; j < descriptors_scene.rows; j++ ){ 
						if( matches[j].distance < 3*min_dist ){
							if(matches[j].queryIdx > 0 && matches[j].queryIdx < keypoints_track[i].size()){
								good_matches.push_back( matches[j]);
							} 
						}
 					}

					//draw matches - This will only work with one image to track
					//drawMatches( img_track[k], keypoints_track[k], img_scene, keypoints_scene, good_matches[k], img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    			//Get the keypoints from the good matches
 					for( int j = 0; j < good_matches.size(); j++ ){
  		  		track.push_back(keypoints_track[i][good_matches[j].queryIdx].pt );
  		  		scene.push_back(keypoints_scene[good_matches[j].trainIdx].pt );
  				}

					//Get transform using the matches
					if(track.size() > 4 && scene.size() > 4){
						H = cv::findHomography( track, scene, CV_RANSAC );

  					track_corners[0] = cvPoint(0,0); 
						track_corners[1] = cvPoint(imgs_track[i].cols, 0 );
  					track_corners[2] = cvPoint(imgs_track[i].cols, imgs_track[i].rows );
						track_corners[3] = cvPoint( 0, imgs_track[i].rows );

						perspectiveTransform( track_corners, scene_corners, H);
						
						areaTransform = cv::contourArea(scene_corners);
						
						//check if bounded area makes sense as a match
						if(areaTransform > 100){
							bounding = cv::boundingRect(scene_corners);
							areaBounding = bounding.area();
							areaNorm = floor((areaTransform/areaBounding)*100);

							if(areaNorm > foundThresh){
								cv::line( img_print, scene_corners[0], scene_corners[1], color[i%color.size()], 4 );
  							cv::line( img_print, scene_corners[1], scene_corners[2], color[i%color.size()], 4 );
  							cv::line( img_print, scene_corners[2], scene_corners[3], color[i%color.size()], 4 );
  							cv::line( img_print, scene_corners[3], scene_corners[0], color[i%color.size()], 4 );
								
								//print foundChar
								if(output){
									ss.str("");
									ss << foundChar << "found" << i << ", ";
									temp = ss.str();
									foundChar = (char*)temp.c_str();
									found = true;
								}
							}
						}

					//----These draw lines are for if you have the draw matches function uncommented
					/*cv::line( img_scene, scene_corners[0] + cv::Point2f( img_track.cols, 0), scene_corners[1] + cv::Point2f( img_track.cols, 0), color[i%color.size()], 4 );
  				cv::line( img_scene, scene_corners[1] + cv::Point2f( img_track.cols, 0), scene_corners[2] + cv::Point2f( img_track.cols, 0), color[i%color.size()], 4 );
  				cv::line( img_scene, scene_corners[2] + cv::Point2f( img_track.cols, 0), scene_corners[3] + cv::Point2f( img_track.cols, 0), color[i%color.size()], 4 );
  				cv::line( img_scene, scene_corners[3] + cv::Point2f( img_track.cols, 0), scene_corners[0] + cv::Point2f( img_track.cols, 0), color[i%color.size()], 4 );*/

						track.clear();
						track.resize(4);
						scene.clear();
						scene.resize(4);

						keypoints_scene.clear();
					}

					if(output){
						if(!found){
							ss.str("");
							ss << foundChar << ", ";
							temp = ss.str();
							foundChar = (char*)temp.c_str();
							found = false;
						}
					}				
				}//--end of tracking loop
			}

			if(output){
				//get timestamp
				ss.str("");
				currSecs = ros::Time::now().toSec() - startSecs;
				ss << foundChar << currSecs;
				temp = ss.str();
				foundChar = (char*)temp.c_str();
				
				file << foundChar << std::endl;

				ss.str("");
				ss << currSecs;
				temp = ss.str();
				timeStamp = (char*)temp.c_str();

				cv::putText(img_print, foundChar, cv::Point(0, img_print.rows), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0));

				ss.str("");
				ss << path << arg << timeStamp << ".jpg";
				temp = ss.str();
				writePath = (char*)temp.c_str();

				cv::imwrite( writePath, img_print );
			}

			//Show Image
			cv::imshow("view", img_print);
			cv::waitKey(5);
    }
  	ros::spinOnce();
  }
	file.close();
  cv::destroyWindow("view");
}
