// STD
#include <experimental/filesystem>
#include <chrono>
#include <sstream>
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
  bool has_data = false;
  std::string topic_name;
  double speed = 1.0;
  int width, height, img_num = 0;
  if(argc<3){
    std::cout << "\033[1;31mInsufficient input arguments\n\
Usage: ./bag_to_video bag_name output_video_name [speed]\n\033[0m";
    exit(EXIT_FAILURE);
  }
  std::string in_bag(argv[1]),
              out_name(argv[2]);
  if(out_name.length()<=4) // .mp4
    out_name+=".mp4";
  else{
    if(out_name.substr(out_name.length()-4, 4)!=".mp4")
      out_name+=".mp4";
  }
  if(argc==4)
    speed = atof(argv[3]);
  std::cout << "Input bag: " << in_bag << "\n";
  std::cout << "Output video: " << out_name << "\n";
  std::cout << "Video speed: " << speed << "\n";
  rosbag::Bag bag;
  try{
    bag.open(in_bag, rosbag::bagmode::Read);
  } catch(const rosbag::BagIOException& e){
    std::cout << "\033[1;31mProvided bag name unopenable, please make sure you provide correct name, exiting...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  rosbag::View view(bag);
  cv_bridge::CvImagePtr cv_bridge_ptr;
  double bag_duration = (view.getEndTime() - view.getBeginTime()).toSec();
  std::cout << "Bag duration: " << bag_duration << " seconds\n";
  // Get number of frame
  std::cout << "Counting frames: \n";
  auto s_ts = std::chrono::high_resolution_clock::now();
  std::experimental::filesystem::create_directory("tmp");  // Temporary directory for saving images
  foreach(const rosbag::MessageInstance m, view){
    if(m.getDataType()=="sensor_msgs/Image"){
      sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
      try{
        cv_bridge_ptr = cv_bridge::toCvCopy(img_ptr);
      } catch(cv_bridge::Exception& e){
        std::cout << "\033[1;33mcv_bridge exception: " << e.what() << "\033[0m\n";
        exit(EXIT_FAILURE);
      }
      // Make sure is color image topic
      if(cv_bridge_ptr->encoding=="rgb8" and not has_data){ // ROS uses RGB
        has_data = true;
        topic_name = m.getTopic();
      }
      // Make sure is the same topic
      if(m.getTopic()!=topic_name) continue;
      double read_ratio = ((m.getTime() - view.getBeginTime()).toSec())/bag_duration;
      width = img_ptr->width;
      height = img_ptr->height;
      cv::Mat rgb_img;
      cv::cvtColor(cv_bridge_ptr->image, rgb_img, CV_BGR2RGB);
      std::stringstream ss; ss.width(6); ss.fill('0'); ss << img_num;
      std::string img_name = "tmp/" + ss.str() + ".jpg";
      cv::imwrite(img_name, rgb_img);
      printf("\r[%05.1f%%] %d", read_ratio*100, img_num+1);
      ++img_num;
    }
  }
  if(!has_data){
    std::cout << "\033[1;31mNo color image detected in provided bag, abort...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  std::cout << "\nNum of frames: " << img_num << "\nFPS: " << img_num/bag_duration << "\nCounting down...\n";
  cv::VideoWriter video(out_name, CV_FOURCC('H', '2', '6', '4'), img_num/bag_duration*speed, cv::Size(width, height));
  for(int i=0; i<img_num; ++i){
    std::stringstream ss; ss.width(6); ss.fill('0'); ss << i;
    std::string img_name = "tmp/" + ss.str() + ".jpg";
    cv::Mat img = cv::imread(img_name);
    if(!img.empty()){
      video << img;
      printf("\r[%d/%d]", i+1, img_num);
      remove(img_name.c_str());
    }
  }
  std::cout << "\n";
  std::experimental::filesystem::remove("tmp");
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  std::cout << "Conversion time: " << duration*1e-9 << " seconds\n";
  return 0;
}
