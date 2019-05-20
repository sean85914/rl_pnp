#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_path_node");
  ros::NodeHandle nh("~");
  if(argc!=2){
    ROS_ERROR("No input file specific, exiting...");
    return -1;
  }
  std::ifstream f;
  f.open(argv[1], std::ifstream::in);
  if(!f){
    ROS_ERROR("Cannot open file!");
    return -1;
  }
  std::string line;
  nav_msgs::Path path;
  path.header.frame_id = "base_link";
  while(std::getline(f, line)){
    std::stringstream ss(line);
    geometry_msgs::PoseStamped p;
    ss >> p.pose.position.x 
       >> p.pose.position.y
       >> p.pose.position.z;
    path.poses.push_back(p);
  }
  ROS_INFO("There are %d points in file, start publishing...", (int)path.poses.size());
  ros::Publisher pub = nh.advertise<nav_msgs::Path>("path", 1);
  while(ros::ok()){
    pub.publish(path);
  }
  pub.shutdown();
  f.close();
  return 0;
}
