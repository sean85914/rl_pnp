#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h> // Information and state of gripper
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h> // Command from user

int upper_thres; // Gripper position(PO) greater than this will consider as fail grasp
int lower_thres; 
std::string node_name;
ros::Publisher pub_gripper_out;
ros::Subscriber sub_gripper_in;
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input now_state;

// Initial robotiq 2-finger gripper, refer to: package:robotiq_2f_gripper_control/nodes/Robotiq2FGripperSimpleController.py
void initial_gripper(void){
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
  pub_gripper_out.publish(msg);// Reset
  ros::Duration(0.5).sleep();
  msg.rACT = 1; msg.rGTO = 1; msg.rSP = 200; msg.rFR = 100;
  pub_gripper_out.publish(msg); // Active
  ROS_INFO("[%s] Robotiq gripper initialization complete!", node_name.c_str());
}
// Subscriber callback, only update the global variable for future usage
void cb_update_state(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input msg){
  now_state = msg;
}
// Close gripper service callback
bool close_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("[%s] Receive close gripper request!", node_name.c_str());
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
  msg.rACT = 1; msg.rGTO = 1; msg.rPR = 255; msg.rSP = 200; msg.rFR = 100;
  pub_gripper_out.publish(msg);
  return true;
}
// Open gripper service callback
bool open_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("[%s] Receive open gripper request!", node_name.c_str());
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
  msg.rACT = 1; msg.rGTO = 1; msg.rPR = 0; msg.rSP = 200; msg.rFR = 100;
  pub_gripper_out.publish(msg);
  return true;
}
// Check if the action is success
bool get_grasp_state_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  ROS_INFO("[%s] Now gripper position: %d", node_name.c_str(), now_state.gPO);
  if(now_state.gPO >= upper_thres or now_state.gPO <= lower_thres){
    res.success = false;
    res.message = "Fail to grasp object";
  } else{ // Gripper position now less than the threshold will be considered as succeed
    res.success = true;
    res.message = "Successfully grasp object";
  }
}
// Initial gripper
bool initial_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  initial_gripper();
  return true;
}

void timer_cb(const ros::TimerEvent &event){
  int lower, upper;
  ros::param::get(node_name+"/lower_thres", lower); 
  if(lower!=lower_thres){
    ROS_INFO("[%s] lower_thres changes from %d to %d", node_name.c_str(), lower_thres, lower);
    lower_thres = lower;
  }
  ros::param::get(node_name+"/upper_thres", upper); 
  if(upper!=upper_thres){
    ROS_INFO("[%s] upper_thres changes from %d to %d", node_name.c_str(), upper_thres, upper);
    upper_thres = upper;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotiq_finger_control_node");
  ros::NodeHandle nh, pnh("~");
  node_name = ros::this_node::getName();
  // Setup publisher and subscriber
  pub_gripper_out = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
  sub_gripper_in  = nh.subscribe("Robotiq2FGripperRobotInput", 1, cb_update_state);
  // Get parameters
  if(!pnh.getParam("upper_thres", upper_thres)) upper_thres = 222;
  if(!pnh.getParam("lower_thres", lower_thres)) lower_thres = 10;
  ROS_INFO("[%s] Gripper valid PO range: (%d, %d)", node_name.c_str(), lower_thres, upper_thres);
  // Initialize gripper
  ros::Duration(0.5).sleep(); initial_gripper();
  // Advertise services
  ros::ServiceServer close_gripper_service     = pnh.advertiseService("close_gripper", close_gripper_cb);
  ros::ServiceServer open_gripepr_service      = pnh.advertiseService("open_gripper", open_gripper_cb);
  ros::ServiceServer check_grasp_succeed       = pnh.advertiseService("get_grasp_state", get_grasp_state_cb);
  ros::ServiceServer initial_gripper_service   = pnh.advertiseService("initial_gripper", initial_gripper_cb);
  ros::Timer param_checker                     = pnh.createTimer(ros::Duration(1.0), timer_cb);
  // Spin until ROS shutdown  
  ros::spin();
  return 0;
}
