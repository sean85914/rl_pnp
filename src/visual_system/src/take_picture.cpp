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
  int num;
  std::string package_path; // Package path string
  std::string file_path; // File will be saved in this directory
  cv_bridge::CvImagePtr cv_ptr;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer saveImageServer;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_img;
  void cb_img(const sensor_msgs::ImageConstPtr&);
  bool cb_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
 public:
  ImageSaver(ros::NodeHandle, ros::NodeHandle);
};

ImageSaver::ImageSaver(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), it(nh), num(0){
  saveImageServer = pnh_.advertiseService("save_image", &ImageSaver::cb_service, this);
  sub_img = it.subscribe("image_raw", 1, &ImageSaver::cb_img, this);
  package_path = ros::package::getPath("visual_system");
  // Check if images directory exist, if not, then create one
  file_path = package_path + "/images";
  boost::filesystem::path p(file_path);
  if(!boost::filesystem::exists(p)){
    ROS_WARN("images directory doesn't exist, create one.");
    boost::filesystem::create_directory(p);
  }
}

void ImageSaver::cb_img(const sensor_msgs::ImageConstPtr &msg){
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
}

bool ImageSaver::cb_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  if(cv_ptr!=NULL){
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6);
    ss << num;
    std::string file_name = ss.str();
    file_name = file_path + "/" + file_name + ".jpg";
    if(cv::imwrite(file_name, cv_ptr->image)){
      ROS_INFO("%s saved!", file_name.c_str());
      ++num;
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "take_picture_service_server_node");
  ros::NodeHandle nh, pnh("~");
  ImageSaver foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
