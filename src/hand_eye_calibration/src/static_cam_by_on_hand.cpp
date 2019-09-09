#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

class static_cam_calibration{
 private:
  const int REQURIED_DATA = 4;
  int data_num;
  bool completed;
  std::fstream fs;
  std::string camera_name;
  std::string tag_name;
  std::string file_name;
  std::string package_path;
  std::string file_path;
  ros::NodeHandle nh_, pnh_;
  tf::TransformListener listener;
  void write_data(tf::Vector3 p_tcp, tf::Vector3 p_tag){
    fs << p_tcp.getX() << " "
       << p_tcp.getY() << " "
       << p_tcp.getZ() << " "
       << p_tag.getX() << " "
       << p_tag.getY() << " "
       << p_tag.getZ() << "\n";
  }
  
 public:
  static_cam_calibration(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), completed(false), data_num(0)
  {
    if(!pnh_.getParam("camera_name", camera_name)) camera_name = "camera";
    if(!pnh_.getParam("tag_name", tag_name)) tag_name = "tag_0";
    if(!pnh_.getParam("file_name", file_name)) file_name = "calibration";
    ROS_INFO("\n*********************************\n \
camera_name: %s\n \
tag_name: %s\n \
file_name: %s\n \
*********************************", camera_name.c_str(), tag_name.c_str(), file_name.c_str());
    package_path = ros::package::getPath("hand_eye_calibration");
    file_path = package_path + "/data/" + file_name + ".txt";
    // Check if data directory exist
    boost::filesystem::path p(package_path+"/data");
    if(!boost::filesystem::exists(p)){
      ROS_INFO("Directory doesnot exist, creating one...");
      boost::filesystem::create_directory(p);
    }
    fs.open(file_path, std::fstream::out | std::fstream::app); // Write file at the end
    if(!fs.is_open()) {ROS_ERROR("Cannot open file!"); ros::shutdown();}
  }
  void printInfo(void){
    ROS_INFO("At least %d data required, you have %d data logged.\n \
Press 'r' to record data, 'e' to exit the process:", REQURIED_DATA, data_num);
    char command; std::cin >> command;
    if((command == 'e' or command == 'E') and data_num < REQURIED_DATA){
      ROS_WARN("Not enough data, do you sure you want to exit?\n [Press 'y' to exit]");
      char sure; std::cin >> sure;
      if(sure == 'y' or sure == 'Y') {fs.close(); ros::shutdown(); completed = true;}
    } else if(command == 'r' or command == 'R'){
      tf::StampedTransform stf_tcp, stf_tag;
      try{
        listener.waitForTransform("base_link", tag_name, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("base_link", tag_name, ros::Time(0), stf_tcp);
        listener.waitForTransform(camera_name+"_link", tag_name, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(camera_name+"_link", tag_name, ros::Time(0), stf_tag);
        write_data(stf_tcp.getOrigin(), stf_tag.getOrigin());
        ++data_num;
      } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what());}	
    } else if(command == 'e' or command == 'E'){
      ROS_INFO("%d data collected, regular shutdown...", data_num); fs.close(); ros::shutdown(); completed = true;
    }else {ROS_WARN("Invalid input, ignore...");}
  }
  bool getStatus(void) {return completed;}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_cam_calibration");
  ros::NodeHandle nh, pnh("~");
  static_cam_calibration Foo(nh, pnh);
  while(ros::ok() and !Foo.getStatus()) Foo.printInfo();
  return 0;
}
