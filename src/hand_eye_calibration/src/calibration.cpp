#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <arm_operation/target_pose.h>

class calibration{
 private:
  int count;
  double dx, dz;
  std::fstream fs;
  std::string package_path;
  std::string file_path;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceClient client;
  geometry_msgs::Pose original_pose;
  arm_operation::target_pose req;
  tf::TransformListener listener;
  void write_data(geometry_msgs::Pose p_tcp, geometry_msgs::Pose p_tag){
    fs << p_tcp.position.x << " "
       << p_tcp.position.y << " "
       << p_tcp.position.z << "|\t"
       << p_tag.position.x << " "
       << p_tag.position.y << " "
       << p_tag.position.z << "\n";
  }
  bool get_tag_data(tf::StampedTransform &t){
    try{
      listener.waitForTransform("camera_link", "tag_0", ros::Time(0), ros::Duration(2.0));
      listener.lookupTransform("camera_link", "tag_0", ros::Time(0), t);
    } catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return 0;
    }
    return 1;
  }
  bool get_arm_data(tf::StampedTransform &t, std::string target){
    try{
      listener.waitForTransform("base_link", target, ros::Time(0), ros::Duration(0.5));
      listener.lookupTransform("base_link", target, ros::Time(0), t);
    } catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return 0;
    }
    return 1;
  }
  geometry_msgs::Pose transform2Pose(tf::StampedTransform t){
    geometry_msgs::Pose res;
    res.position.x = t.getOrigin().getX();
    res.position.y = t.getOrigin().getY();
    res.position.z = t.getOrigin().getZ();
    res.orientation.x = t.getRotation().getX();
    res.orientation.y = t.getRotation().getY();
    res.orientation.z = t.getRotation().getZ();
    res.orientation.w = t.getRotation().getW();
    return res;
  }
  void calibration_process(void){
    while(count<10){
      ROS_INFO("----------- %d -----------", count);
      switch(count){
        case 0:
          req.request.target_pose = original_pose;
          break;
        case 1:
          req.request.target_pose.position.z -= dz;
          break;
        case 2:
          req.request.target_pose.position.x += dx;
          break;
        case 3:
          req.request.target_pose.position.z += dz;
          break;
        case 4:
          req.request.target_pose.position.x += dx;
          break;
        case 5:
          req.request.target_pose.position.z -= dz;
          break;
        case 6:
          req.request.target_pose.position.x += dx;
          break;
        case 7:
          req.request.target_pose.position.z += dz;
          break;
        case 8:
          req.request.target_pose.position.x += dx;
          break;
        case 9:
          req.request.target_pose.position.z -= dz;
          break;
      }
      client.call(req);
      ros::Duration(1.0).sleep();
      tf::StampedTransform st;
      get_tag_data(st);
      geometry_msgs::Pose p_tag = transform2Pose(st);
      get_arm_data(st, "tcp_link");
      geometry_msgs::Pose p_tcp = transform2Pose(st);
      write_data(p_tcp, p_tag);
      ++count;
    }
    ROS_INFO("Go home...");
    req.request.target_pose = original_pose;
    client.call(req);
    ros::Duration(1.0).sleep();
  }
  
 public:
  calibration(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), count(0){
    req.request.factor = 0.5; 
    package_path = ros::package::getPath("arm_operation");
    file_path = package_path + "/data/calibration.txt";
    fs.open(file_path, std::fstream::out | std::fstream::app); // Write file
    if(!fs.is_open()) {ROS_ERROR("Cannot open file!"); ros::shutdown();}
    if(!pnh_.getParam("dx", dx)) dx = 0.03; ROS_INFO("dx: %f", dx);
    if(!pnh_.getParam("dz", dz)) dz = 0.05; ROS_INFO("dz: %f", dz);
    ros::service::waitForService("/ur3_control_server/ur_control/go_straight"); ROS_INFO("Service connect!");
    client = nh_.serviceClient<arm_operation::target_pose>("/ur3_control_server/ur_control/goto_pose", true);
    ROS_INFO("Start process");
    tf::StampedTransform transform;
    get_arm_data(transform, "ee_link");
    original_pose = transform2Pose(transform);
    fs << "Robot  position: |\t\t tag position\n";
    calibration_process();
    ROS_INFO("End process");
    ros::shutdown();
  }
  ~calibration(){
    fs.close();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh, pnh("~");
  calibration foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}

