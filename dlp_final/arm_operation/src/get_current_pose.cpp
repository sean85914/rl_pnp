#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

/* 
 * Type 1: 
 * 
 * Save the joint angles of specific waypoints to a given file
 *
 * Subscribe:
 *   /joint_states
 * Parameter:
 *   ~type: execution type
 *   ~file_name: file name, excluding its extension, the file will be saved in directory package://arm_operation/data/
 * 
 * Command from user: 
 *   press \s\ to save waypoint, \e\ to exit
 * 
 */
 
/*
 * Type 2:
 * 
 * Save \tcp_link\ position w.r.t \base_link\ consecutively
 *
 * Parameter:
 *   ~type: execution type
 *   ~file_name: file name, excluding its extension, the file will be saved in directory package://arm_operation/data/
 *
 * Command from user:
 *   press \s\ to start saving, CTRL+C to stop
 */

// Last modify: 5/18, 2019
// Editor: Sean
// TODO: make sure there are six joint position

sensor_msgs::JointState js;
std::fstream fp;

void cb_js(const sensor_msgs::JointState msg)
{
  if(msg.name.size() != 6){
    ROS_INFO("Incorrect vector size, ignore...");
    return;
  }
  js = msg;
}

void type_1(std::fstream &fp){
  char to_do;
  while(ros::ok()){
    std::cout << "Press 's' to save the waypoint, 'e' to exit the process:\n";
    std::cin >> to_do;
    if(to_do == 'e'){
      ROS_INFO("End process.");
      break;
    }
    else if(to_do == 's'){
      ros::spinOnce(); // Update data
      if(js.position.size() != 6) {ROS_WARN("Joint state size incorrect, ignore..."); continue;}
      for(int i=0;i<6;++i){
        fp << js.position[i] << " ";
      }
      fp << std::endl;
      ROS_INFO("Waypoint saved.");
    }
    else
      ROS_ERROR("No such input.");
  }
}

void type_2(std::fstream &fp){
  char to_do;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::cout << "Press 's' to start saving. After recording, press CTRL+C to exit the process:\n";
  std::cin >> to_do;
  while(ros::ok()){
    try{
      listener.waitForTransform("base_link", "tcp_link", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform("base_link", "tcp_link", ros::Time(0), transform);
    } catch(tf::TransformException &ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // Record data
    fp << transform.getOrigin().getX() << " "
       << transform.getOrigin().getY() << " "
       << transform.getOrigin().getZ() << "\n"; std::cout << "Data recorded!\n";
    ros::Duration(0.2).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_current_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  int type; // execution type
  std::string package_path = ros::package::getPath("arm_operation"),
              file_name, // file name from parameter server
              file_str; // file_str = package_path + "/data/" + file_name + ".txt"
  if(!ros::param::get("~type", type)) type = 1; ROS_INFO("type: %d", type);
  if(!ros::param::get("~file_name", file_name)) file_name = "data"; ROS_INFO("file_name: %s", file_name.c_str());
  if(type == 1){
    sub = nh.subscribe("/joint_states", 10, cb_js);
  }
  // Check if folder exist
  boost::filesystem::path p(package_path+"/data");
  if(!boost::filesystem::exists(p)){ // Not exists, create one
    boost::filesystem::create_directory(p);
  }
  file_str = package_path + "/data/" + file_name + ".txt";
  fp.open(file_str.c_str(), std::fstream::out); // Will erase content in specific file
  if(!fp){
    ROS_ERROR("Can't open file.");
    return -1;
  }
  switch(type){
    case 1: type_1(fp); break;
    case 2: type_2(fp); break;
  }
  // Close file
  fp.close();
  ros::shutdown();
  return 0;
}
