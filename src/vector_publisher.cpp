#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "precision_land/PosError.h"


ros::Publisher pub;
image_transport::Subscriber sub;


// Target center
float TargetX, TargetY;

//Image center
float ImageX, ImageY;

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        
        cv::Point TargetCenter;

        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Error between Image and Mark
        float ErX = 0.0;
        float ErY = 0.0;

        // Get the Image center
        ImageX = InImage.cols / 2.0f;
        ImageY = InImage.rows / 2.0f;

        // Create the msg
        precision_land::PosError msg;

	//convert bgr to hsv
	cv::Mat imgHSV;
	cv::cvtColor(InImage,imgHSV,cv::COLOR_BGR2HSV);

	cv::Mat lower_red_hue_range;
   	cv::Mat upper_red_hue_range;
	cv::Mat target_image;

  	cv::inRange(imgHSV, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
  	cv::inRange(imgHSV, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, target_image);
	
	cv::Mat imgThresholded = target_image;

  	//morphological opening (removes small objects from the foreground)
  	cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  	cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   	//morphological closing (removes small holes from the foreground)
  	cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  	cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  	//Calculate the moments of the thresholded image
  	cv::Moments oMoments = cv::moments(imgThresholded);

  	double dM01 = oMoments.m01;
 	double dM10 = oMoments.m10;
  	double dArea = oMoments.m00;
	
  	if (dArea > 10000)
  	{
   	//calculate the position of the ball
   	TargetCenter.x = dM10 / dArea;
   	TargetCenter.y = dM01 / dArea;
  	}
	
	circle(imgThresholded,TargetCenter,10,Scalar( 100, 100, 100 ),-1,8);

	ErX = ImageX - TargetCenter.x;
	ErY = ImageY - TargetCenter.y;

	msg.errx = ErX;
	msg.erry = ErY;
        pub.publish(msg);

	imshow("view", imgThresholded);
	waitKey(30);
	
	
	ROS_INFO("Error = (%f , %f)", ErX, ErY);
	
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculate_pose_error");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    pub = nh.advertise<precision_land::PosError>("errors_in_pos", 10);;
    ros::spin();
    
    return 0;
}
