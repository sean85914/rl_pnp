#include <ros/ros.h>
#include <tf/tf.h>
// Server headers
#include <arm_operation/change_tool.h>
#include <abb_node/robot_GetCartesian.h>
#include <abb_node/robot_SetCartesian.h>
#include <abb_node/robot_GetJoints.h>
#include <abb_node/robot_SetJoints.h>
#include <abb_node/robot_SetZone.h>
#include <abb_node/robot_SetSpeed.h>
#include <std_srvs/SetBool.h> // Vacuum
#include <std_srvs/Empty.h> // Go home and go place
#include <arm_operation/agent_abb_action.h>

void setTargetPose(abb_node::robot_SetCartesian& srv, geometry_msgs::Point position, tf::Quaternion quat){
  srv.request.cartesian[0] = position.x*1000.0; // x, m to mm
  srv.request.cartesian[1] = position.y*1000.0; // y, m to mm
  srv.request.cartesian[2] = position.z*1000.0; // z, m to mm
  srv.request.quaternion[0] = quat.getW(); // q0, a.k.a qw
  srv.request.quaternion[1] = quat.getX(); // q1, a.k.a qx
  srv.request.quaternion[2] = quat.getY(); // q2, a.k.a qy
  srv.request.quaternion[3] = quat.getZ(); // q3, a.k.a qz
}

class AgentServer{
 private:
  int curr_tool_id; // ID of current tool, from parameter server
  const double tool_head_length = 0.555f; // Length of spear tool head, roughly measured
  std::vector<double> tool_length; // Length of three tool, from parameter server
  std::vector<double> home_joints; // Joints for home pose
  std::vector<double> middle_joints; // Middle joint for placing
  std::vector<double> place_joints; // Joints for placing pose
  std::vector<std::string> service_name_vec;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceClient change_tool_client;
  ros::ServiceClient get_cartesian_client;
  ros::ServiceClient set_cartesian_client;
  ros::ServiceClient get_joints_client;
  ros::ServiceClient set_joints_client;
  ros::ServiceClient set_zone_client;
  ros::ServiceClient set_speed_client;
  ros::ServiceClient vacuum_control_client;
  ros::ServiceServer agent_action_server;
  ros::ServiceServer go_home_server;
  ros::ServiceServer go_place_server;
  void setupParams(void);
  void setupServiceClients(void);
  void getInitialToolID(void);
  bool homeCB(std_srvs::Empty::Request&,
              std_srvs::Empty::Response&);
  bool placeCB(std_srvs::Empty::Request&,
               std_srvs::Empty::Response&);
  bool serviceCB(arm_operation::agent_abb_action::Request&, 
                 arm_operation::agent_abb_action::Response&);
 public:
  AgentServer(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
    tool_length.resize(3);
    home_joints.resize(6);
    middle_joints.resize(6);
    place_joints.resize(6);
    service_name_vec.resize(8);
    setupParams();
    // Wait all services to arise
    for(int i=0; i<service_name_vec.size(); ++i){
      while(ros::ok() and !ros::service::waitForService(service_name_vec[i], ros::Duration(3.0))){
        ROS_WARN("Waiting for service: %s to arise...", service_name_vec[i].c_str());
      }
    }
    setupServiceClients();
    agent_action_server = pnh_.advertiseService("agent_take_action", &AgentServer::serviceCB, this);
    go_home_server      = pnh_.advertiseService("go_home", &AgentServer::homeCB, this);
    go_place_server     = pnh_.advertiseService("go_place", &AgentServer::placeCB, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agent_server_node");
  ros::NodeHandle nh, pnh("~");
  AgentServer foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}

void AgentServer::setupParams(void){
  if(!nh_.getParam("curr_tool_id", curr_tool_id)){
    getInitialToolID();
  }else{
    ROS_INFO("current tool id: %d", curr_tool_id);
  }
  if(!pnh_.getParam("tool_length", tool_length)){
    ROS_ERROR("No tool lengths given, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }else{
    ROS_INFO("\nTool 1 length: %f\nTool 2 length: %f\nTool 3 length: %f", tool_length[0], tool_length[1], tool_length[2]);
  }
  if(!pnh_.getParam("service_name", service_name_vec)){
    ROS_ERROR("No service name vector given, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
  if(!pnh_.getParam("home_joints", home_joints)){
    ROS_ERROR("No home joints provided, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }else{
    ROS_INFO("Home joints: [%f, %f, %f, %f, %f, %f]", home_joints[0], home_joints[1], home_joints[2], home_joints[3], home_joints[4], home_joints[5]);
  }
  if(!pnh_.getParam("middle_joints", middle_joints)){
    ROS_ERROR("No middle joints provided, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }else{
    ROS_INFO("Middle joints: [%f, %f, %f, %f, %f, %f]", middle_joints[0], middle_joints[1], middle_joints[2], middle_joints[3], middle_joints[4], middle_joints[5]);
  }
  if(!pnh_.getParam("place_joints", place_joints)){
    ROS_ERROR("No place joints provided, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }else{
    ROS_INFO("Place joints: [%f, %f, %f, %f, %f, %f]", place_joints[0], place_joints[1], place_joints[2], place_joints[3], place_joints[4], place_joints[5]);
  }
}

void AgentServer::getInitialToolID(void){
  ROS_INFO("\033[1;31mNo current_tool_id parameter set, please input the ID: (-1, 1, 2, 3)\033[0m");
  bool valid_input = false;
  do{
    std::cin >> curr_tool_id;
    if(curr_tool_id==-1 or curr_tool_id==1 or curr_tool_id==2 or curr_tool_id==3){
      ROS_INFO("curr_tool_id set to %d", curr_tool_id);
      nh_.setParam("curr_tool_id", curr_tool_id);
      valid_input = true;
    }
    else{
      ROS_WARN("Invalid number, please input again");
    }
  } while(!valid_input and ros::ok());
}

void AgentServer::setupServiceClients(void){
  change_tool_client = pnh_.serviceClient<arm_operation::change_tool>(service_name_vec[0]);
  get_cartesian_client = pnh_.serviceClient<abb_node::robot_GetCartesian>(service_name_vec[1]);
  set_cartesian_client = pnh_.serviceClient<abb_node::robot_SetCartesian>(service_name_vec[2]);
  get_joints_client = pnh_.serviceClient<abb_node::robot_GetJoints>(service_name_vec[3]);
  set_joints_client = pnh_.serviceClient<abb_node::robot_SetJoints>(service_name_vec[4]);
  set_zone_client = pnh_.serviceClient<abb_node::robot_SetZone>(service_name_vec[5]);
  set_speed_client = pnh_.serviceClient<abb_node::robot_SetSpeed>(service_name_vec[6]);
  vacuum_control_client = pnh_.serviceClient<std_srvs::SetBool>(service_name_vec[7]);
}

bool AgentServer::serviceCB(arm_operation::agent_abb_action::Request&  req, 
                            arm_operation::agent_abb_action::Response& res){
  ros::Time ts = ros::Time::now();
  if(req.tool_id!=1 and req.tool_id!=2 and req.tool_id!=3){
    ROS_WARN("Invalid tool ID given, abort...");
    res.result = "Invalid ID given";
    return true;
  }
  if(curr_tool_id!=req.tool_id){
    // Change tool
    arm_operation::change_tool change_tool_req;
    change_tool_req.request.now = curr_tool_id;
    change_tool_req.request.togo = req.tool_id;
    change_tool_client.call(change_tool_req);
    curr_tool_id = req.tool_id;
    nh_.setParam("curr_tool_id", curr_tool_id);
  }
  tf::Quaternion quat(0.0, 1.0, 0.0, 0.0), // [-1, 0, 0; 0, 1, 0; 0, 0, -1]
                 quat_compensate;
  quat_compensate.setRPY(0.0, 0.0, req.angle);
  quat *= quat_compensate;
  geometry_msgs::Point target_ee_position(req.position);
  target_ee_position.z += (tool_length[req.tool_id-1] + tool_head_length + 0.2);
  // Get current pose and store it into buffer
  abb_node::robot_GetCartesian get_cartesian;
  get_cartesian_client.call(get_cartesian);
  // Go to first waypoint, 20 cm above target position
  abb_node::robot_SetCartesian set_cartesian;
  set_cartesian.request.cartesian.resize(3); set_cartesian.request.quaternion.resize(4);
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  ros::Duration(0.5).sleep();
  // Turn on vacuum suction
  std_srvs::SetBool bool_data; bool_data.request.data = true;
  vacuum_control_client.call(bool_data);
  // Set zone to fine
  abb_node::robot_SetZone set_zone;
  set_zone.request.mode = 0;
  set_zone_client.call(set_zone);
  // Go to target, downward 2 cm
  target_ee_position.z -= 0.22;
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  ros::Duration(0.5).sleep();
  // Set zone to z0
  set_zone.request.mode = 1;
  set_zone_client.call(set_zone);
  // Go to first waypoint
  target_ee_position.z += 0.22;
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  ros::Duration(0.5).sleep();
  // Go back to original pose
  geometry_msgs::Point original_position;
  original_position.x = get_cartesian.response.x/1000.0;
  original_position.y = get_cartesian.response.y/1000.0;
  original_position.z = get_cartesian.response.z/1000.0;
  tf::Quaternion original_orientation(get_cartesian.response.qx, get_cartesian.response.qy, get_cartesian.response.qz, get_cartesian.response.q0);
  setTargetPose(set_cartesian, original_position, original_orientation);
  set_cartesian_client.call(set_cartesian);
  double time = (ros::Time::now()-ts).toSec();
  res.result = "success, action takes " + std::to_string(time) + " seconds"; 
  return true;
}

bool AgentServer::homeCB(std_srvs::Empty::Request&,
                         std_srvs::Empty::Response&){
  ros::Time ts = ros::Time::now();
  abb_node::robot_SetJoints set_joints;
  set_joints.request.position.resize(6);
  set_joints.request.position.assign(home_joints.begin(), home_joints.end());
  set_joints_client.call(set_joints);
  ROS_INFO("Go home takes %f seconds", (ros::Time::now()-ts).toSec());
  return true;
}

bool AgentServer::placeCB(std_srvs::Empty::Request&,
                         std_srvs::Empty::Response&){
  ros::Time ts = ros::Time::now();
  abb_node::robot_SetZone set_zone;
  abb_node::robot_SetSpeed set_speed;
  // Set zone to z_10, speed to (300, 100)
  set_speed.request.tcp = 300.0f;
  set_speed.request.ori = 100.0f;
  set_zone.request.mode = 4; // z_10
  set_zone_client.call(set_zone);
  set_speed_client.call(set_speed);
  abb_node::robot_SetJoints set_joints;
  set_joints.request.position.resize(6);
  // First, go to middle joints
  set_joints.request.position.assign(middle_joints.begin(), middle_joints.end());
  set_joints_client.call(set_joints);
  // Second, go to place joints
  set_joints.request.position.assign(place_joints.begin(), place_joints.end());
  set_joints_client.call(set_joints);
  ros::Duration(0.5).sleep();
  // Then turn off suction
  std_srvs::SetBool bool_data;
  bool_data.request.data = false;
  vacuum_control_client.call(bool_data);
  // Set zone to z_0
  set_zone.request.mode = 1;
  set_zone_client.call(set_zone);
  // And then go back
  set_joints.request.position.assign(home_joints.begin(), home_joints.end());
  set_joints_client.call(set_joints);
  // Speed to original one
  set_speed.request.tcp = 200.0f;
  set_speed.request.ori = 100.0f;
  set_speed_client.call(set_speed);
  ROS_INFO("Go place takes %f seconds", (ros::Time::now()-ts).toSec());
  return true;
}
