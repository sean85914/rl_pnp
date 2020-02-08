// STD
#include <chrono>
// Boost
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
  std::vector<cv::Mat> img_vec;
  cv_bridge::CvImagePtr cv_bridge_ptr;
  double bag_duration = (view.getEndTime() - view.getBeginTime()).toSec();
  std::cout << "Bag duration: " << bag_duration << " seconds\n";
  // Get number of frame
  int img_num = 0, width, height;
  std::cout << "Counting frames: \n";
  auto s_ts = std::chrono::high_resolution_clock::now();
  foreach(const rosbag::MessageInstance m, view){
    if(m.getDataType()=="sensor_msgs/Image"){
      ++img_num;
      printf("\r%d", img_num);
      sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
      width = img_ptr->width;
      height = img_ptr->height;
      try{
        cv_bridge_ptr = cv_bridge::toCvCopy(img_ptr);
      } catch(cv_bridge::Exception& e){
        std::cout << "\033[1;33mcv_bridge exception: " << e.what() << "\033[0m\n";
        exit(EXIT_FAILURE);
      }
      cv::Mat rgb_img;
      cv::cvtColor(cv_bridge_ptr->image, rgb_img, CV_BGR2RGB);
      img_vec.push_back(rgb_img);
    }
  }
  std::cout << "\nNum of frames: " << img_num << "\nFPS: " << img_num/bag_duration << "\nCounting down...\n";
  std::string output_filename(argv[2]);
  output_filename+=".avi";
  cv::VideoWriter video(output_filename, CV_FOURCC('M', 'J', 'P', 'G'), img_num/bag_duration, cv::Size(width, height));
  for(int i=0; i<img_vec.size(); ++i){
    video << img_vec[i];
    printf("\r[%d/%d]", i+1, img_num);
  }
  std::cout << "\n";
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  std::cout << "Conversion time: " << duration*1e-9 << " seconds\n";
  return 0;
}
