#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <arm_operation/joint_pose.h>

/* Read the waypoints file and make UR go to pose of user-specific index in the file
 * Subscribe:
 *   ~go_index (std_msgs::Int16): index to go
 * Service client:
 *   /urX_control_server/ur_control/goto_joint_pose
 * Parameter:
 *   ~type: which robot to use, either 3 or 5
 *   ~file_name: file name with extension
 * Last modify: 7/28, 2019
 * Changelog:
 *  7/28: revise hardcoded service name and add robot choosing parameter, abort invalid files
 * Editor: Sean
 */

class GoToWP {
 private:
  int wp_len;
  double **wp_list;
  // Request
  arm_operation::joint_pose js_req;
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Subscriber
  ros::Subscriber sub_index;
  // Parameters
  bool type; // 0: ur3; 1: ur5
  std::string file_name;
  std::string service_name;
  // Service client
  ros::ServiceClient goto_joint_pose_client_;
  // Parse input file to double matrix array
  bool parse_file(){
    std::string line;
    std::ifstream fp;
    int counter = 0;
    int index = 0;
    double** res = 0;
    fp.open(file_name.c_str(), std::ifstream::in);
    if(!fp){
      ROS_ERROR("Can't open file.");
      return 0;
    }
    while(std::getline(fp, line)){
      ++counter;
    }
    wp_len = counter;
    res = new double*[counter];
    for(int i=0; i<counter; ++i){
      res[i] = new double[6];
    }
    // return the cursor to the begining of the file
    fp.clear();
    fp.seekg(0, std::ios::beg);
    while(!fp.eof()){
      std::getline(fp, line);
      if(line.empty()) break;
      std::stringstream ss(line);
      int i = 0;
      while(ss.good() && i < 6){
        ss >> res[index][i];
        ++i;
      } if(i!=6) {ROS_ERROR("Not enough joints given, please check your file!"); return 0;}
      ++index;
    }
    // Close the file
    fp.close();
    wp_list = res; return 1;
  }
  void cbCallback(const std_msgs::Int16 msg){
    if(msg.data>=wp_len or msg.data<0) {ROS_WARN("Given index out of range, ignore..."); return;}
    for(int i=0; i<6; ++i){
      js_req.request.joint[i] = wp_list[msg.data][i];
    }
    goto_joint_pose_client_.call(js_req); ros::Duration(0.3).sleep();
  }
 public:
  GoToWP(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
    sub_index = pnh_.subscribe("index_to_go", 1, &GoToWP::cbCallback, this);
    if(!pnh_.getParam("type", type)) {type=true;}
    ROS_INFO("%s", (type==0?"Use UR3":"Use UR5"));
    service_name = "/ur" + std::to_string((type==0?3:5)) + "_control_server/ur_control/goto_joint_pose";
    if(!pnh_.getParam("file_name", file_name)) {ROS_ERROR("No file provide, exit..."); ros::shutdown();}
    if(!parse_file()){ros::shutdown(); return;}
    ROS_INFO("Get %d waypoints", wp_len);
    goto_joint_pose_client_ = nh_.serviceClient<arm_operation::joint_pose>(service_name);
    while(!ros::service::waitForService(service_name, ros::Duration(3.0)) and ros::ok()){
      ROS_WARN("Service not available, still waiting...");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_pose");
  ros::NodeHandle nh, pnh("~");
  GoToWP gtwp(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
