#include <boost/foreach.hpp>
// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
// CV
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
  if(argc!=3){
    std::cout << "\033[1;31mInsufficient input arguments\n\
Usage: ./bag_to_video [bag_name] [output_video_name]\n\033[0m\
\033[1;33mNote: output_video_name without extension\n\033[0m";
    exit(EXIT_FAILURE);
  }
  rosbag::Bag bag;
  try{
    bag.open(argv[1], rosbag::bagmode::Read);
  } catch(const rosbag::BagIOException& e){
    std::cout << "\033[1;31mProvided bag name unopenable, please make sure you provide correct name, exiting...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  rosbag::View view(bag);
  double bag_duration = (view.getEndTime() - view.getBeginTime()).toSec();
  // Get number of frame
  int img_num = 0, width, height;
  foreach(const rosbag::MessageInstance m, view){
    if(m.getDataType()=="sensor_msgs/Image"){
      ++img_num;
      sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
      width = img_ptr->width;
      height = img_ptr->height;
    }
  }
  std::string output_filename(argv[2]);
  output_filename+=".avi";
  cv::VideoWriter video(output_filename, CV_FOURCC('M', 'J', 'P', 'G'), img_num/bag_duration, cv::Size(width, height));
  cv_bridge::CvImagePtr cv_bridge_ptr;
  foreach(const rosbag::MessageInstance m, view){
    if(m.getDataType()=="sensor_msgs/Image"){
      sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
      try{
        cv_bridge_ptr = cv_bridge::toCvCopy(img_ptr);
      } catch(cv_bridge::Exception& e){
        std::cout << "\033[1;33mcv_bridge exception: " << e.what() << "\033[0m\n";
        exit(EXIT_FAILURE);
      }
      cv::Mat rgb_img;
      cv::cvtColor(cv_bridge_ptr->image, rgb_img, CV_BGR2RGB);
      video << rgb_img;
    }
  }
  return 0;
}
