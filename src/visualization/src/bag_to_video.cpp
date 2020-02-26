// STD
#include <experimental/filesystem>
#include <chrono>
#include <utility>
#include <thread>
// Boost
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
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

typedef std::pair<cv::Mat, ros::Time> data_pair;
bool read_complete = false;
int width, height;
double speed = 1.0;
std::string out_name;
boost::shared_mutex _access;

void write_video(std::vector<data_pair> &data_vec){
  while(data_vec.size()<2){usleep(10000);} // Wait until more than 2 data
  double estimated_fps = 1/(data_vec[1].second - data_vec[0].second).toSec();
  cv::VideoWriter video(out_name, CV_FOURCC('H', '2', '6', '4'), estimated_fps*speed, cv::Size(width, height));
  while(true){
    if(read_complete and data_vec.empty()) break; // Stop
    if(data_vec.empty()) continue; // Consume too fast
    video << data_vec[0].first;
    data_vec.erase(data_vec.begin());
    if(read_complete){
      std::cout << "\r" << data_vec.size() << " images remaining...";
    }
  }
}


int main(int argc, char** argv)
{
  // Parse input argument
  bool has_data = false;
  std::string topic_name="";
  if(argc<3){
    std::cout << "\033[1;31mInsufficient input arguments\n\
Usage: ./bag_to_video bag_name output_video_name [speed]\n\033[0m";
    exit(EXIT_FAILURE);
  }
  std::string in_bag(argv[1]);
  out_name = std::string(argv[2]);
  if(out_name.length()<=4) // .mp4
    out_name+=".mp4";
  else{
    size_t pos;
    out_name.find(".");
    if(pos==std::string::npos){
      out_name+=".mp4";
    }
  }
  if(argc==4)
    speed = atof(argv[3]);
  std::cout << "Input bag: " << in_bag << "\n";
  std::cout << "Output video: " << out_name << "\n";
  std::cout << "Video speed: " << speed << "\n";
  // Open bag
  rosbag::Bag bag;
  try{
    bag.open(in_bag, rosbag::bagmode::Read);
  } catch(const rosbag::BagIOException& e){
    std::cout << "\033[1;31mProvided bag name unopenable, please make sure you provide correct name, exiting...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  rosbag::View view(bag);
  cv_bridge::CvImagePtr cv_bridge_ptr;
  std::vector<data_pair> data_vec;
  double bag_duration = (view.getEndTime() - view.getBeginTime()).toSec();
  std::cout << "Bag duration: " << bag_duration << " seconds\n";
  std::cout << "Converting...\n";
  // Start writing thread
  std::thread writing_thread(write_video, std::ref(data_vec));
  // Start reading
  auto s_ts = std::chrono::high_resolution_clock::now();
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
      printf("\r[%05.1f%%]", read_ratio*100);
      data_pair pair = std::make_pair(rgb_img, m.getTime());
      data_vec.push_back(pair);
    }
  }
  read_complete = true;
  if(topic_name.empty()){
    std::cout << "\033[0;31mNo RGB image detected in the bag, exiting...\n\033[0m";
    exit(EXIT_FAILURE);
  }
  // Wait writing thread to stop
  std::cout << "\nWriting video...\n";
  writing_thread.join();
  std::cout << "Complete!\n";
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  std::cout << "Conversion time: " << duration*1e-9 << " seconds\n";
}
