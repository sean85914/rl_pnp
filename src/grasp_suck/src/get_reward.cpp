// C++ STL
#include <iomanip>
#include <sstream>
#include <cassert>
// CV
#include <opencv2/opencv.hpp> 
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
// SRV
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <grasp_suck/get_result.h>
// MSG
#include <sensor_msgs/Image.h>

inline bool inRange(const int data, const int upper, const int lower){
  if(data>lower and data <upper) return true;
  else return false;
}

int thres; // Number of huge change pixel greater than this number will be considered as success
const int LENGTH = 224;
const int UPPER  = -10; // 1 cm
const int LOWER  = -100;  // 10 cm

bool cb_cal(grasp_suck::get_result::Request &req, grasp_suck::get_result::Response &res);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_action_state");
  ros::NodeHandle pnh("~");
  if(!pnh.getParam("thres", thres)) thres = 180;
  ros::ServiceServer get_result = pnh.advertiseService("get_result", cb_cal);
  ros::spin();
  return 0;
}

bool cb_cal(grasp_suck::get_result::Request &req, grasp_suck::get_result::Response &res)
{
  cv_bridge::CvImagePtr prior_depth, post_depth;
  prior_depth = cv_bridge::toCvCopy(req.prior, sensor_msgs::image_encodings::TYPE_16UC1);
  post_depth  = cv_bridge::toCvCopy(req.post,  sensor_msgs::image_encodings::TYPE_16UC1);
  assert(prior_depth->image.cols == LENGTH);
  int cnt = 0;
  for(int y=0; y<prior_depth->image.cols; ++y){
    for(int x=0; x<prior_depth->image.rows; ++x){
      int prior_val = prior_depth->image.at<unsigned short>(cv::Point(x, y)),
          post_val  = post_depth->image.at<unsigned short>(cv::Point(x, y)),
          diff = post_val - prior_val; // Stands `height from plane`
      if(inRange(diff, UPPER, LOWER)) {++cnt;}
    }
  }
  ROS_INFO("\033[1;33mNumber of difference Pixel: %d\033[0m", cnt);
  if(cnt>thres) {ROS_INFO("\033[1;32mMOVED\033[0m"); res.result.data = true;}
  else {ROS_INFO("\033[1;32mNO MOVED\033[0m"); res.result.data=false;}
  return true;
}
