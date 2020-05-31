/*
 *  Service to collect dataset
 *  Subscribe topic: /image_raw (sensor_msgs::Image)
 *  Advertise service: ~save_image (std_srvs::Empty)
 *    After you call this service, it will save the image in package:visual_system/images
 */

// C++ STL
#include <iomanip>
#include <sstream>
// Boost
#include <boost/filesystem.hpp>
// OpenCV
#include <opencv2/opencv.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// MSG
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// SRV
#include <std_srvs/Empty.h>

class ImageSaver{
 private:
  int color_num, depth_num;
  std::string package_path; // Package path string
  std::string color_file_path, depth_file_path; // File will be saved in this directory
  cv_bridge::CvImagePtr cv_color_ptr, cv_depth_ptr;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer saveColorImageServer, saveDepthImageServer;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_color_img, sub_depth_img;
  void cb_color_img(const sensor_msgs::ImageConstPtr&);
  void cb_depth_img(const sensor_msgs::ImageConstPtr&);
  bool cb_service_color(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool cb_service_depth(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
 public:
  ImageSaver(ros::NodeHandle, ros::NodeHandle);
};

ImageSaver::ImageSaver(ros::NodeHandle nh, ros::NodeHandle pnh): 
 nh_(nh), pnh_(pnh), it(nh), color_num(0), depth_num(0){
  saveColorImageServer = pnh_.advertiseService("save_color_image", &ImageSaver::cb_service_color, this);
  saveDepthImageServer = pnh_.advertiseService("save_depth_image", &ImageSaver::cb_service_depth, this);
  sub_color_img = it.subscribe("color_image_raw", 1, &ImageSaver::cb_color_img, this);
  sub_depth_img = it.subscribe("depth_image_raw", 1, &ImageSaver::cb_depth_img, this);
  package_path = ros::package::getPath("visual_system");
  // Check if images directory exist, if not, then create one
  color_file_path = package_path + "/images/color/";
  depth_file_path = package_path + "/images/depth/";
  boost::filesystem::path p(package_path+"/images"), p_color(color_file_path), p_depth(depth_file_path);
  if(!boost::filesystem::exists(p)){ROS_WARN("images directory doesn't exist, create one."); boost::filesystem::create_directory(p);}
  if(!boost::filesystem::exists(p_color)){
    ROS_WARN("Color images directory doesn't exist, create one.");
    boost::filesystem::create_directory(p_color);
  }
  if(!boost::filesystem::exists(p_depth)){
    ROS_WARN("Depth image directory doesn't exist, create one.");
    boost::filesystem::create_directory(p_depth);
  }
}

// Subscriber callback
void ImageSaver::cb_color_img(const sensor_msgs::ImageConstPtr &msg){
  try{
    cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
}

void ImageSaver::cb_depth_img(const sensor_msgs::ImageConstPtr &msg){
  try{
    cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
}

// Service callback
bool ImageSaver::cb_service_color(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  if(cv_color_ptr!=NULL){
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6);
    ss << color_num;
    std::string file_name = ss.str();
    file_name = color_file_path + file_name + ".jpg";
    if(cv::imwrite(file_name, cv_color_ptr->image)){
      ROS_INFO("%s saved!", file_name.c_str());
      ++color_num;
      return true;
    }else {
      ROS_ERROR("Image save failed!");
      return false;
    }
  } else{
    ROS_ERROR("No image received yet!");
    return false;
  }
}

bool ImageSaver::cb_service_depth(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  if(cv_depth_ptr!=NULL){
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6);
    ss << depth_num;
    std::string file_name = ss.str();
    file_name = depth_file_path + file_name + ".png"; // JPG support only 8-bit, use PNG for 16-bit
    if(cv::imwrite(file_name, cv_depth_ptr->image)){
      ROS_INFO("%s saved!", file_name.c_str());
      ++depth_num;
      return true;
    }else {
      ROS_ERROR("Image save failed!");
      return false;
    }
  } else{
    ROS_ERROR("No image received yet!");
    return false;
  }
}

/*
  __  __      _      ___   _   _ 
 |  \/  |    / \    |_ _| | \ | |
 | |\/| |   / _ \    | |  |  \| |
 | |  | |  / ___ \   | |  | |\  |
 |_|  |_| /_/   \_\ |___| |_| \_|
                                 
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "take_picture_service_server_node");
  ros::NodeHandle nh, pnh("~");
  ImageSaver foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
