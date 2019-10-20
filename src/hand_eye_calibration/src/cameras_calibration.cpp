#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Point stampedTransform2Point(tf::StampedTransform t){
  geometry_msgs::Point p;
  p.x = t.getOrigin().getX();
  p.y = t.getOrigin().getY();
  p.z = t.getOrigin().getZ();
  return p;
}

class CamerasCalibration{
 private:
  int data_count; // Count how many data recorded
  int tag_mount; // Mount of tags
  std::string master_cam_name; // Master camera prefix/name, from parameter server
  std::string master_cam_link; // Master camera prefix + "_link"
  std::string slave_cam_name; // Slave camera prefix/name, from parameter server
  std::string slave_cam_link; // Slave camera prefix + "_link"
  std::string package_path; // Path to this package
  std::string file_name; // Saved file name, from parameter server
  std::string file_path; // Path to saved file, package path + folder + file name + file extension
  std::fstream fs;
  ros::NodeHandle nh_, pnh_;
  void printParams(void);
  void writeData(geometry_msgs::Point, geometry_msgs::Point);
  bool getTransform(std::string, std::string, tf::StampedTransform&);
 public:
  CamerasCalibration(ros::NodeHandle, ros::NodeHandle);
  bool getInput(void);
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cameras_calibration");
  ros::NodeHandle nh, pnh("~");
  CamerasCalibration foo(nh, pnh);
  while(!foo.getInput()) ros::spinOnce();
  return 0;
}

CamerasCalibration::CamerasCalibration(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), data_count(0){
  if(!pnh_.getParam("tag_mount", tag_mount)) tag_mount = 5;
  if(!pnh_.getParam("master_cam_name", master_cam_name)) master_cam_name = "camera1";
  if(!pnh_.getParam("slave_cam_name", slave_cam_name)) slave_cam_name = "camera2";
  if(!pnh_.getParam("file_name", file_name)) file_name = "cameras_calibration";
  printParams();
  master_cam_link = master_cam_name+"_link";
  slave_cam_link  = slave_cam_name +"_link";
  package_path = ros::package::getPath("hand_eye_calibration");
  boost::filesystem::path p(package_path+"/data");
  if(!boost::filesystem::exists(p)) boost::filesystem::create_directory(p);
  file_path = package_path + "/data/" + file_name + ".txt";
  fs.open(file_path, std::fstream::out | std::fstream::app); // Write file at the end
  ROS_INFO("[%s] Node ready", ros::this_node::getName().c_str());
}

void CamerasCalibration::printParams(void){
  std::stringstream ss;
  ss << "\ntag_mount: " << tag_mount << "\n"
     << "master_cam_name: " << master_cam_name << "\n"
     << "slave_cam_name: "  << slave_cam_name  << "\n"
     << "file_name: " << file_name;
  ROS_INFO("[%s] %s", ros::this_node::getName().c_str(), ss.str().c_str());
}

void CamerasCalibration::writeData(geometry_msgs::Point p_master, geometry_msgs::Point p_slave){
  fs << p_master.x << " "
     << p_master.y << " "
     << p_master.z << " "
     << p_slave.x  << " "
     << p_slave.y  << " "
     << p_slave.z  << "\n";
}

bool CamerasCalibration::getTransform(std::string target, std::string source , tf::StampedTransform &t){
  static tf::TransformListener listener;
  try{
    listener.waitForTransform(target, source, ros::Time(0), ros::Duration(2.0f));
    listener.lookupTransform(target, source, ros::Time(0), t);
    return true;
  }catch(tf::TransformException ex){
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), ex.what());
    return false;
  }
}

bool CamerasCalibration::getInput(void){
  ROS_INFO("[%s] You have %d data in the file\n\
Press `r` to record current data, and press `e` to exit", 
ros::this_node::getName().c_str(), data_count);
  char in;
  std::cin >> in;
  tf::StampedTransform t_master, t_slave;
  if(in=='e'){
    fs.close();
    return true; // Finish
  } 
  else if(in=='r'){
    for(int i=0; i<tag_mount; ++i){
      std::string tag_frame = "tag_" + std::to_string(i);
      if(!getTransform(master_cam_link, tag_frame, t_master) or 
         !getTransform(slave_cam_link,  tag_frame, t_slave)){
         ROS_WARN("[%s] Can't get transform, make sure both cameras see the tag, abort request", ros::this_node::getName().c_str());
      }else{
        auto p_master = stampedTransform2Point(t_master);
        auto p_slave  = stampedTransform2Point(t_slave);
        writeData(p_master, p_slave);
        ROS_INFO("[%s] One data wrote into the file", ros::this_node::getName().c_str());
        ++data_count;
      }
    }
  } else{
    ROS_WARN("[%s] Invalid input, abort", ros::this_node::getName().c_str());
  }
  return false;
}
