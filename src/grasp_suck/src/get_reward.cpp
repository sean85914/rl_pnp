// C++ STL
#include <iomanip>
#include <sstream>
// Boost
#include <boost/filesystem.hpp>
// CV
#include <opencv2/opencv.hpp> 
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>


inline bool inRange(const int data, const int upper, const int lower){
  if(data>lower and data <upper) return true;
  else return false;
}

bool prior_ready = false;
bool post_ready  = false;
bool img_ready   = false;
int req_cnt = 0;
int thres; // Number of huge change pixel greater than this number will be considered as success
const int UPPER  = -10; // 1 cm
const int LOWER  = -100;  // 10 cm
const int X_MIN  = 208;
const int Y_MIN  = 21;
const int LENGTH = 224;
std::string file_path = ros::package::getPath("grasp_suck") + "/images";

cv::Mat prior, post;
cv::Rect myROI(X_MIN, Y_MIN, LENGTH, LENGTH);
cv_bridge::CvImagePtr cv_color_ptr, cv_depth_ptr;

//void cb_color_img(const sensor_msgs::ImageConstPtr& msg);
void cb_depth_img(const sensor_msgs::ImageConstPtr& msg);
bool cb_prior(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
bool cb_post(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
bool cb_cal(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

int main(int argc, char** argv)
{
  boost::filesystem::path p(file_path);
  if(!boost::filesystem::exists(p)){ROS_WARN("images directory doesn't exist, create one."); boost::filesystem::create_directory(p);}
  ros::init(argc, argv, "get_action_state");
  ros::NodeHandle pnh("~");
  if(!pnh.getParam("thres", thres)) thres = 180;
  ros::ServiceServer setPriorServer = pnh.advertiseService("set_prior", cb_prior);
  ros::ServiceServer setPosteriorServer = pnh.advertiseService("set_posterior", cb_post);
  ros::ServiceServer get_result = pnh.advertiseService("get_result", cb_cal);
  ros::Subscriber sub_depth_img = pnh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, cb_depth_img);
  //ros::Subscriber sub_color_img = pnh.subscribe("/camera/color/image_raw", 1, cb_color_img);
  ros::spin();
  return 0;
}

void cb_depth_img(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    img_ready = true;
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
}

/*void cb_color_img(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
}*/

bool cb_prior(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  prior = cv_depth_ptr->image(myROI);
  prior_ready = true; img_ready = false;
  /*std::stringstream ss;
  ss << std::setfill('0') << std::setw(6);
  ss << req_cnt;
  std::string file_name = ss.str();
  file_name = file_path + "/prior_" + file_name + ".jpg";
  cv::imwrite(file_name, cv_color_ptr->image(myROI));*/
  ROS_INFO("Prior ready!");
  return true;
}

bool cb_post(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  if(!prior_ready) {ROS_INFO("Prior not ready yet!"); return false;}
  post = cv_depth_ptr->image(myROI);
  post_ready = true; img_ready = false;
  /*std::stringstream ss;
  ss << std::setfill('0') << std::setw(6);
  ss << req_cnt;
  std::string file_name = ss.str();
  file_name = file_path + "/post_" + file_name + ".jpg";
  cv::imwrite(file_name, cv_color_ptr->image(myROI));*/
  ROS_INFO("Posterior ready!");
  return true;
}

bool cb_cal(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  if(prior_ready and post_ready){
    int cnt = 0;
    for(int y = 0; y<prior.cols; ++y){
      for(int x = 0; x<prior.rows; ++x){
        int priorVal = prior.at<unsigned short>(cv::Point(x, y)),
            postVal  = post.at<unsigned short>(cv::Point(x, y)),
            diff = priorVal - postVal;
        if(inRange(diff, UPPER, LOWER)) {++cnt;}
      }
    }
    ROS_INFO("\033[1;33mNumber of difference Pixel: %d\033[0m", cnt);
    if(cnt>thres) {ROS_INFO("\033[1;32mMOVED\033[0m"); res.success = true;}
    else {ROS_INFO("\033[1;32mNO MOVED\033[0m"); res.success=false;}
    prior_ready = false; post_ready = false; ++req_cnt;
    return true;
  }else{
    ROS_WARN("Image not ready yet!");
    return false;
  }
}
