#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"
#include "track_pkg/Box.h"
#include <std_msgs/Bool.h>


static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 0.4
#define Min_linear_speed 0.1
#define Min_distance 1.0
#define Max_distance 5.0
#define Max_rotation_speed 0.75

float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.002;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1 = 200;
int ERROR_OFFSET_X_left2 = 600;
int ERROR_OFFSET_X_right1 = 680;
int ERROR_OFFSET_X_right2 = 1080;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool tracking_obj = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker obj_detect
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  ros::Subscriber enable_sub_;
  
public:
    ros::Publisher end_pub_;
  ros::Publisher pub;
  bool is_tracking_ = false;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
      &ImageConverter::depthCb, this);
    pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    enable_sub_ = nh_.subscribe("/tracker/enable", 1,
            &ImageConverter::enableCb, this);
    end_pub_ = nh_.advertise<std_msgs::Bool>("/tracker/end", 1);

    cv::namedWindow(RGB_WINDOW);
    //cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    //cv::destroyWindow(DEPTH_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!is_tracking_)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.copyTo(rgbimage);

//    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        // if (selectRect.width <= 0 || selectRect.height <= 0)
        // {
        //     bRenewROI = false;
        //     //continue;
        // }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
        enable_get_depth = true;
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

//    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!is_tracking_)
        return;
  	cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  		cv_ptr->image.copyTo(depthimage);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  	}

    if(enable_get_depth)
    {
      dist_val[0] = depthimage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
      dist_val[1] = depthimage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
      dist_val[2] = depthimage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
      dist_val[3] = depthimage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
      dist_val[4] = depthimage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;

      float distance = 0;
      int num_depth_points = 5;
      for(int i = 0; i < 5; i++)
      {
        if(dist_val[i] > 400.0 && dist_val[i] < 8000.0)
          distance += dist_val[i];
        else
          num_depth_points--;
      }
      distance /= 1000.0;
      distance /= num_depth_points;

      if (std::isnan(distance)){
          distance = 0;
      }

      //calculate linear speed
      if(distance > Min_distance)
        linear_speed = distance * k_linear_speed + h_linear_speed;
      else
        linear_speed = 0;

      if(linear_speed > Max_linear_speed)
        linear_speed = Max_linear_speed;

      //calculate rotation speed
      int center_x = result.x + result.width/2;
      if(center_x < ERROR_OFFSET_X_left1) 
        rotation_speed =  Max_rotation_speed;
      else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
      else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
      else if(center_x > ERROR_OFFSET_X_right2)
        rotation_speed = -Max_rotation_speed;
      else 
        rotation_speed = 0;

      std::cout <<  "linear_speed = " << linear_speed << "  rotation_speed = " << rotation_speed << std::endl;

      std::cout <<  dist_val[0]  << " / " <<  dist_val[1] << " / " << dist_val[2] << " / " << dist_val[3] <<  " / " << dist_val[4] << std::endl;
      std::cout <<  "distance = " << distance << std::endl;
    }

  	//cv::imshow(DEPTH_WINDOW, depthimage);
  	cv::waitKey(1);
  }

  void enableCb(const track_pkg::Box &box){
      std::string people = "people";
      ROS_INFO("obj_name is %s", box.obj_name.c_str());
      if ((box.obj_name.c_str() == people) == 1){
          ROS_INFO("tracking people");
          tracking_obj = false;
      } else{
          ROS_INFO("tracking obj");
          tracking_obj = true;
      }
      std::cout << "enable tracking" << std::endl;
      selectRect.x = box.xmin;
      selectRect.y = box.ymin;
      selectRect.width = box.xmax - box.xmin;
      selectRect.height = box.ymax - box.ymin;
      cv_bridge::toCvCopy(box.image, sensor_msgs::image_encodings::BGR8)->image.copyTo(rgbimage);
      selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
      is_tracking_ = true;
      bRenewROI = true;
  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
	ImageConverter ic;

	int stop_counts = 0;
	std_msgs::Bool send_false;
  
	while(ros::ok())
	{
		ros::spinOnce();

    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed; 
    twist.linear.y = 0; 
    twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = rotation_speed;
    if (tracking_obj){
        twist.linear.x = 0;
    }
    if (ic.is_tracking_){
        ic.pub.publish(twist);

        if (linear_speed == 0 && rotation_speed == 0){
            stop_counts += 1;
        } else{
            stop_counts = 0;
        }

        if (stop_counts > 20){
            std::cout << "finish tracker" << std::endl;
            ROS_INFO("finish_tracker");
            ic.is_tracking_ = false;
            select_flag = false;
            bRenewROI = false;
            bBeginKCF = false;
            send_false.data = false;
            ROS_INFO("%d", send_false.data);
            enable_get_depth = false;
            cv::destroyAllWindows();
            ic.end_pub_.publish(send_false);
        }
    }

		if (cvWaitKey(33) == 'q')
      break;
	}

	return 0;
}

