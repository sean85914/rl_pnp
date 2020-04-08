#include <ros/ros.h>
#include <tf/tf.h>
// MSG
#include <geometry_msgs/Point.h>
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
#include <arm_operation/execution.h>
#include <arm_operation/publish_info.h>

bool in_range(double data, double lower, double upper){
  if(data<=upper and data>=lower) return true;
  else
    return false;
}

void setTargetPose(abb_node::robot_SetCartesian& srv, geometry_msgs::Point position, tf::Quaternion quat){
  srv.request.cartesian[0] = position.x*1000.0; // x, m to mm
  srv.request.cartesian[1] = position.y*1000.0; // y, m to mm
  srv.request.cartesian[2] = position.z*1000.0; // z, m to mm
  srv.request.quaternion[0] = quat.getW(); // q0, a.k.a qw
  srv.request.quaternion[1] = quat.getX(); // q1, a.k.a qx
  srv.request.quaternion[2] = quat.getY(); // q2, a.k.a qy
  srv.request.quaternion[3] = quat.getZ(); // q3, a.k.a qz
}

void printQuat(const tf::Quaternion quat){
  std::cout << quat.getX() << " "
            << quat.getY() << " "
            << quat.getZ() << " "
            << quat.getW() << "\n";
}

void printMat(const tf::Matrix3x3 mat){
  for(int i=0; i<3; ++i){
    std::cout << mat.getColumn(i).getX() << " "
              << mat.getColumn(i).getY() << " "
              << mat.getColumn(i).getZ() << "\n";
  } std::cout << "\n";
}

class AgentServer{
 private:
  int curr_tool_id; // ID of current tool, from parameter server
  int count[2] = {0}; // Execution counter, home and place
  int action_count[3] = {0};  // Gripper, bagcup suction, small suction
  double mean_time[2] = {0.0}; // Execution mean time, indice as above
  double action_time[3] = {0.0}; 
  const double tool_head_length = 0.555f; // Length of spear tool head, roughly measured
  std::vector<double> tool_length; // Length of three tool, from parameter server
  std::vector<double> home_joints; // Joints for home pose
  std::vector<double> home_xyz; // XYZ coordinate of home
  std::vector<double> middle_joints; // Middle joint for placing
  std::vector<double> place_joints; // Joints for placing pose
  std::vector<std::string> service_name_vec;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_execution_info;
  ros::ServiceClient change_tool_client;
  ros::ServiceClient get_cartesian_client;
  ros::ServiceClient set_cartesian_client;
  ros::ServiceClient get_joints_client;
  ros::ServiceClient set_joints_client;
  ros::ServiceClient set_zone_client;
  ros::ServiceClient set_speed_client;
  ros::ServiceClient vacuum_control_client;
  ros::ServiceClient check_suck_success_client;
  ros::ServiceServer agent_action_server;
  ros::ServiceServer go_home_server;
  ros::ServiceServer go_place_server;
  ros::ServiceServer go_home_fix_orientation_server;
  ros::ServiceServer fast_vibrate_server;
  ros::ServiceServer light_vibrate_server;
  ros::ServiceServer publish_data_server;
  ros::ServiceServer check_if_collide_server;
  void setupParams(void);
  void setupServiceClients(void);
  void getInitialToolID(void);
  void _home(void);
  void _home_fix_ori(void);
  void _fast_vibrate(void);
  void _light_vibrate(void);
  void joint_6_coterminal(abb_node::robot_SetJoints&);
  double get_final_TCP_height(const int, const double);
  bool homeCB(std_srvs::Empty::Request&,
              std_srvs::Empty::Response&);
  bool placeCB(std_srvs::Empty::Request&,
               std_srvs::Empty::Response&);
  bool serviceCB(arm_operation::agent_abb_action::Request&, 
                 arm_operation::agent_abb_action::Response&);
  bool home_fixCB(std_srvs::Empty::Request&,
                  std_srvs::Empty::Response&);
  bool fast_vibrateCB(std_srvs::Empty::Request&,
                      std_srvs::Empty::Response&);
  bool light_vibrateCB(std_srvs::Empty::Request&,
                       std_srvs::Empty::Response&);
  bool publishCB(arm_operation::publish_info::Request&,
                 arm_operation::publish_info::Response&);
  bool check_if_collision(arm_operation::agent_abb_action::Request&, 
                          arm_operation::agent_abb_action::Response&);
 public:
  AgentServer(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
    tool_length.resize(3);
    home_xyz.resize(3);
    home_joints.resize(6);
    middle_joints.resize(6);
    place_joints.resize(6);
    service_name_vec.resize(9);
    setupParams();
    // Wait all services to arise
    for(int i=0; i<service_name_vec.size(); ++i){
      while(ros::ok() and !ros::service::waitForService(service_name_vec[i], ros::Duration(3.0))){
        ROS_WARN("Waiting for service: %s to arise...", service_name_vec[i].c_str());
      }
    }
    setupServiceClients();
    pub_execution_info = pnh_.advertise<arm_operation::execution>("execution_info", 1);
    agent_action_server = pnh_.advertiseService("agent_take_action", &AgentServer::serviceCB, this);
    go_home_server      = pnh_.advertiseService("go_home", &AgentServer::homeCB, this);
    go_place_server     = pnh_.advertiseService("go_place", &AgentServer::placeCB, this);
    go_home_fix_orientation_server = pnh_.advertiseService("go_home_fix_orientation", &AgentServer::home_fixCB, this);
    fast_vibrate_server = pnh_.advertiseService("fast_vibrate", &AgentServer::fast_vibrateCB, this);
    light_vibrate_server = pnh_.advertiseService("light_vibrate", &AgentServer::light_vibrateCB, this);
    publish_data_server = pnh_.advertiseService("publish_data", &AgentServer::publishCB, this);
    check_if_collide_server = pnh_.advertiseService("check_if_collide", &AgentServer::check_if_collision, this);
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
  if(!pnh_.getParam("home_xyz", home_xyz)){
    ROS_ERROR("No home coordinate given, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }else{
    ROS_INFO("Home X: %f| Y: %f | Z: %f", home_xyz[0], home_xyz[1], home_xyz[2]);
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
  check_suck_success_client = pnh_.serviceClient<std_srvs::SetBool>(service_name_vec[8]);
}

void AgentServer::joint_6_coterminal(abb_node::robot_SetJoints &joint_req){
  abb_node::robot_GetJoints get_joints;
  get_joints_client.call(get_joints);
  double j6_equivalent = (joint_req.request.position[5]>0?joint_req.request.position[5]-2*M_PI:joint_req.request.position[5]+2*M_PI);
  double dist_1 = fabs(get_joints.response.j6-joint_req.request.position[5]),
         dist_2 = fabs(get_joints.response.j6-j6_equivalent);
  if(dist_1>dist_2){ // Choose smaller
    joint_req.request.position[5] = j6_equivalent;
  }
}

bool AgentServer::serviceCB(arm_operation::agent_abb_action::Request&  req, 
                            arm_operation::agent_abb_action::Response& res){
  //ros::Time ts = ros::Time::now(); Change to count after tool changed
  if(req.tool_id!=1 and req.tool_id!=2 and req.tool_id!=3){
    ROS_WARN("Invalid tool ID given, abort...");
    res.result = "Invalid ID given";
    return true;
  }
  abb_node::robot_SetJoints set_joints;
  set_joints.request.position.resize(6);
  set_joints.request.position.assign(home_joints.begin(), home_joints.end());
  joint_6_coterminal(set_joints);
  set_joints_client.call(set_joints);
  nh_.getParam("curr_tool_id", curr_tool_id);
  if((curr_tool_id!=req.tool_id) or (curr_tool_id==1 and req.tool_id==1)){
    // Change tool, or calibrate gripper
    arm_operation::change_tool change_tool_req;
    change_tool_req.request.now = curr_tool_id;
    change_tool_req.request.togo = req.tool_id;
    change_tool_client.call(change_tool_req);
    curr_tool_id = req.tool_id;
    nh_.setParam("curr_tool_id", curr_tool_id);
  }
  ros::Time ts = ros::Time::now();
  ros::Duration(0.5).sleep();
  // Get current pose and store it into buffer
  abb_node::robot_GetCartesian get_cartesian;
  get_cartesian_client.call(get_cartesian);
  double qx = get_cartesian.response.qx,
         qy = get_cartesian.response.qy,
         qz = get_cartesian.response.qz,
         qw = get_cartesian.response.q0;
  tf::Quaternion quat(qx, qy, qz, qw), // Current orientation
                 quat_perpendicular, // Tune to make Z axis perpendicular to the tote
                 quat_compensate;
  //std::cout << "Original quat: "; printQuat(quat);
  tf::Matrix3x3 mat(quat);
  double a_31 = mat.getRow(2).getX(),
         a_32 = mat.getRow(2).getY(),
         a_33 = mat.getRow(2).getZ();
  double roll  = atan2(a_32, -a_33);
  if(fabs(roll)>=M_PI/2)
    roll += (roll>=0?-M_PI:M_PI);
  double pitch = atan2(a_31, -a_32*sin(roll)+a_33*cos(roll));
  if(fabs(pitch)>=M_PI/2)
    pitch += (pitch>=0?-M_PI:M_PI);
  quat_perpendicular.setRPY(roll, pitch, 0);
  quat *= quat_perpendicular; //std::cout << "Quat compensate to: "; printQuat(quat);
  quat_compensate.setRPY(0.0, 0.0, req.angle);
  quat *= quat_compensate;
  geometry_msgs::Point target_ee_position(req.position); 
  target_ee_position.z += (tool_length[req.tool_id-1] + tool_head_length + 0.2);
  // Go to first waypoint, 20 cm above target position
  abb_node::robot_SetCartesian set_cartesian;
  set_cartesian.request.cartesian.resize(3); set_cartesian.request.quaternion.resize(4);
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  ros::Duration(0.5).sleep();
  // Turn on vacuum if using suction cup
  std_srvs::SetBool bool_data; bool_data.request.data = true;
  if(req.tool_id!=1) vacuum_control_client.call(bool_data);
  // Set zone to fine
  abb_node::robot_SetZone set_zone;
  set_zone.request.mode = 0;
  set_zone_client.call(set_zone);
  // If using suction 2, slow down
  abb_node::robot_SetSpeed set_speed;
  if(req.tool_id==2){
    set_speed.request.tcp = 100.0f;
    set_speed.request.ori = 100.0f;
    set_speed_client.call(set_speed);
  }
  // Go to target, downward 3 cm (5 cm for gripper)
  target_ee_position.z -= (req.tool_id==1?0.25:0.23);
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  if(req.tool_id==2){
    set_speed.request.tcp = 200.0f;
    set_speed_client.call(set_speed);
  }
  // Grasp when gripper reach the target
  if(req.tool_id==1) vacuum_control_client.call(bool_data);
  ros::Duration(0.5).sleep();
  // Set zone to z0
  set_zone.request.mode = 1;
  set_zone_client.call(set_zone);
  // Go to first waypoint, and check if success (only for suction cup)
  target_ee_position.z += 0.2;
  setTargetPose(set_cartesian, target_ee_position, quat);
  set_cartesian_client.call(set_cartesian);
  bool is_success;
  if(req.tool_id!=1){
    ros::Duration(0.1).sleep();
    check_suck_success_client.call(bool_data);
    is_success = bool_data.response.success;
    if(is_success){ // If suck success, raise the object up to prevent collsion with the box during placing
      target_ee_position.z = get_cartesian.response.z/1000.0 + 0.2;
      setTargetPose(set_cartesian, target_ee_position, quat);
      set_cartesian_client.call(set_cartesian);
    }
  }else{ // Gripper
    _home(); ros::Duration(0.1).sleep();
    bool need_vibrate = true;
    nh_.getParam("is_train", need_vibrate);
    if(need_vibrate)
      _light_vibrate(); // Vibrate to make object fall if bad grasping 
  }
  // Move slower to prevent droping
  /*abb_node::robot_SetSpeed set_speed;
  set_speed.request.tcp = 150.0f;
  set_speed.request.ori = 100.0f;
  set_speed_client.call(set_speed);*/
  ros::Duration(0.5).sleep();
  double time = (ros::Time::now()-ts).toSec();
  if(action_count[req.tool_id-1]==0)
    action_time[req.tool_id-1] = time;
  else{
    action_time[req.tool_id-1] = (action_count[req.tool_id-1]*action_time[req.tool_id-1]+time)/(action_count[req.tool_id-1]+1);
  }
  action_count[req.tool_id-1] += 1;
  ROS_INFO("Gripper \t Mean: %f \t Count: %d", action_time[0], action_count[0]);
  ROS_INFO("Bagcup Suction \t Mean: %f \t Count: %d", action_time[1], action_count[1]);
  ROS_INFO("Small Suction \t Mean: %f \t Count: %d", action_time[2], action_count[2]);
  res.result = "success, action takes " + std::to_string(time) + " seconds"; 
  return true;
}

void AgentServer::_home(void){
  abb_node::robot_SetCartesian set_cartesian;
  set_cartesian.request.cartesian.resize(3);
  set_cartesian.request.quaternion.resize(4);
  set_cartesian.request.cartesian.assign(home_xyz.begin(), home_xyz.end());
  set_cartesian.request.quaternion[0] = 0.0;
  set_cartesian.request.quaternion[1] = 0.0;
  set_cartesian.request.quaternion[2] = -1.0;
  set_cartesian.request.quaternion[3] = 0.0;
  set_cartesian_client.call(set_cartesian);
  abb_node::robot_GetJoints get_joints;
  get_joints_client.call(get_joints);
  if(fabs(get_joints.response.j6)>=2*M_PI){
    abb_node::robot_SetJoints set_joints;
    set_joints.request.position.resize(6);
    set_joints.request.position[0] = get_joints.response.j1;
    set_joints.request.position[1] = get_joints.response.j2;
    set_joints.request.position[2] = get_joints.response.j3;
    set_joints.request.position[3] = get_joints.response.j4;
    set_joints.request.position[4] = get_joints.response.j5;
    set_joints.request.position[5] = (get_joints.response.j6>0?get_joints.response.j6-2*M_PI:get_joints.response.j6+2*M_PI);
    set_joints_client.call(set_joints);
  }ros::Duration(0.1).sleep();
}

void AgentServer::_home_fix_ori(void){
  ros::Duration(0.1).sleep();
  // Get current cartesian pose
  abb_node::robot_GetCartesian get_cartesian;
  get_cartesian_client.call(get_cartesian);
  abb_node::robot_SetCartesian set_cartesian;
  set_cartesian.request.cartesian.resize(3); set_cartesian.request.quaternion.resize(4);
  set_cartesian.request.cartesian.assign(home_xyz.begin(), home_xyz.end());
  set_cartesian.request.quaternion[0] = get_cartesian.response.q0;
  set_cartesian.request.quaternion[1] = get_cartesian.response.qx;
  set_cartesian.request.quaternion[2] = get_cartesian.response.qy;
  set_cartesian.request.quaternion[3] = get_cartesian.response.qz;
  set_cartesian_client.call(set_cartesian);
}

void AgentServer::_fast_vibrate(void){
  // Get current cartesian pose
  abb_node::robot_GetCartesian get_cartesian;
  get_cartesian_client.call(get_cartesian);
  // Set high speed
  abb_node::robot_SetSpeed set_speed;
  set_speed.request.tcp = 900.0f; set_speed.request.ori = 100.0f;
  set_speed_client.call(set_speed);
  abb_node::robot_SetCartesian set_cartesian;
  // Move up 5 cm
  set_cartesian.request.cartesian.resize(3); set_cartesian.request.quaternion.resize(4);
  set_cartesian.request.cartesian[0] = get_cartesian.response.x;
  set_cartesian.request.cartesian[1] = get_cartesian.response.y;
  set_cartesian.request.cartesian[2] = get_cartesian.response.z + 50.0;
  set_cartesian.request.quaternion[0] = get_cartesian.response.q0;
  set_cartesian.request.quaternion[1] = get_cartesian.response.qx;
  set_cartesian.request.quaternion[2] = get_cartesian.response.qy;
  set_cartesian.request.quaternion[3] = get_cartesian.response.qz;
  set_cartesian_client.call(set_cartesian);
  // Then move down 5 cm
  set_cartesian.request.cartesian[2] = get_cartesian.response.z - 100.0;
  set_cartesian_client.call(set_cartesian);
  // Then return to original pose and speed
  set_cartesian.request.cartesian[2] = get_cartesian.response.z;
  set_cartesian_client.call(set_cartesian);
  set_speed.request.tcp = 200.0f;
  set_speed_client.call(set_speed);
}

void AgentServer::_light_vibrate(void){
  ros::Duration(0.05).sleep();
  // Set low speed
  abb_node::robot_SetSpeed set_speed;
  set_speed.request.tcp = 25.0f; set_speed.request.ori = 50.0f;
  set_speed_client.call(set_speed);
  // Get current cartesian pose
  abb_node::robot_GetCartesian get_cartesian;
  get_cartesian_client.call(get_cartesian);
  abb_node::robot_SetCartesian set_cartesian;
  set_cartesian.request.cartesian.resize(3); set_cartesian.request.quaternion.resize(4);
  set_cartesian.request.cartesian[0] = get_cartesian.response.x;
  set_cartesian.request.cartesian[1] = get_cartesian.response.y;
  set_cartesian.request.cartesian[2] = get_cartesian.response.z;
  double small_angle = 3.0/180.0*M_PI;
  // Rotate X small angle
  tf::Quaternion current(get_cartesian.response.qx, get_cartesian.response.qy, get_cartesian.response.qz, get_cartesian.response.q0),
                 rotate, target;
  rotate.setRPY(small_angle, 0.0, 0.0); target = current*rotate;
  set_cartesian.request.quaternion[0] = target.getW();
  set_cartesian.request.quaternion[1] = target.getX();
  set_cartesian.request.quaternion[2] = target.getY();
  set_cartesian.request.quaternion[3] = target.getZ();
  set_cartesian_client.call(set_cartesian); ros::Duration(0.05).sleep();
  // Rotate back
  /*rotate.setRPY(-small_angle, 0.0, 0.0); target = current*rotate;
  set_cartesian.request.quaternion[0] = target.getW();
  set_cartesian.request.quaternion[1] = target.getX();
  set_cartesian.request.quaternion[2] = target.getY();
  set_cartesian.request.quaternion[3] = target.getZ();
  set_cartesian_client.call(set_cartesian); ros::Duration(0.05).sleep();*/
  // Recovery to original pose and speed
  set_cartesian.request.quaternion[0] = get_cartesian.response.q0;
  set_cartesian.request.quaternion[1] = get_cartesian.response.qx;
  set_cartesian.request.quaternion[2] = get_cartesian.response.qy;
  set_cartesian.request.quaternion[3] = get_cartesian.response.qz;
  set_cartesian_client.call(set_cartesian);
  set_speed.request.tcp = 200.0f; set_speed.request.ori = 100.0f;
  set_speed_client.call(set_speed);
}

double AgentServer::get_final_TCP_height(const int tool_id, const double target_z){
  double tcp_height;
  if(tool_id!=1){ // Suction
    tcp_height = target_z + tool_length[tool_id-1] + tool_head_length - 0.03;
  }else{
    tcp_height = target_z + tool_length[tool_id-1] + tool_head_length - 0.05;
  }
  return tcp_height;
}

bool AgentServer::homeCB(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res){
  ros::Time ts = ros::Time::now();
  _home();
  if(count[0]==0)
    mean_time[0] = (ros::Time::now()-ts).toSec();
  else{
    mean_time[0] = (count[0]*mean_time[0]+(ros::Time::now()-ts).toSec())/(count[0]+1);
  }
  count[0] += 1;
  ROS_INFO("Go home takes %f seconds | Mean %f | Count %d", (ros::Time::now()-ts).toSec(), mean_time[0], count[0]);
  
  return true;
}

bool AgentServer::home_fixCB(std_srvs::Empty::Request  &req,
                             std_srvs::Empty::Response &res)
{
  ros::Time ts = ros::Time::now();
  _home_fix_ori();
  ROS_INFO("Go home with fixed orientation takes %f seconds", (ros::Time::now()-ts).toSec());
  return true;
}

bool AgentServer::placeCB(std_srvs::Empty::Request  &req,
                          std_srvs::Empty::Response &res){
  ros::Time ts = ros::Time::now();
  abb_node::robot_SetZone set_zone;
  abb_node::robot_SetSpeed set_speed;
  // Set zone to z_10, speed to (150, 100) (slow then fast)
  set_speed.request.tcp = 150.0f;
  set_speed.request.ori = 100.0f;
  set_zone.request.mode = 4; // z_10
  set_zone_client.call(set_zone);
  set_speed_client.call(set_speed);
  abb_node::robot_SetJoints set_joints;
  set_joints.request.position.resize(6);
  // First, go to middle joints
  set_joints.request.position.assign(middle_joints.begin(), middle_joints.end());
  set_joints_client.call(set_joints);
  set_speed.request.tcp = 300.0f;
  set_speed_client.call(set_speed);
  set_joints.request.position.assign(place_joints.begin(), place_joints.end());
  set_joints_client.call(set_joints);
  ros::Duration(0.5).sleep();
  // Then turn off suction
  std_srvs::SetBool bool_data;
  bool_data.request.data = false;
  vacuum_control_client.call(bool_data);
  int curr_tool_id; nh_.getParam("curr_tool_id", curr_tool_id);
  /*if(curr_tool_id==1){
    _fast_vibrate();
  }*/
  // Set zone to z_0
  set_zone.request.mode = 1;
  set_zone_client.call(set_zone);
  // And then middle joints and go home
  if(curr_tool_id==1){
    set_joints.request.position.assign(middle_joints.begin(), middle_joints.end());
    joint_6_coterminal(set_joints);
    set_joints_client.call(set_joints);
  }
  _home();
  // Speed to original one
  set_speed.request.tcp = 200.0f;
  set_speed.request.ori = 100.0f;
  set_speed_client.call(set_speed);
  if(count[1]==0)
    mean_time[1] = (ros::Time::now()-ts).toSec();
  else{
    mean_time[1] = (count[1]*mean_time[1]+(ros::Time::now()-ts).toSec())/(count[1]+1);
  }
  count[1] += 1;
  ROS_INFO("Go place takes %f seconds | Mean %f | Count %d", (ros::Time::now()-ts).toSec(), mean_time[1], count[1]);
  return true;
}

bool AgentServer::fast_vibrateCB(std_srvs::Empty::Request&,
                                 std_srvs::Empty::Response&){
  ros::Time ts = ros::Time::now();
  _fast_vibrate();
  ROS_INFO("Fast vibrate takes %f seconds", (ros::Time::now()-ts).toSec());
  return true;
}

bool AgentServer::light_vibrateCB(std_srvs::Empty::Request  &req,
                                  std_srvs::Empty::Response &res){
  ros::Time ts = ros::Time::now();
  _light_vibrate();
  ROS_INFO("Light vibrate takes %f seconds", (ros::Time::now()-ts).toSec());
  return true;
}

bool AgentServer::publishCB(arm_operation::publish_info::Request &req,
                            arm_operation::publish_info::Response &res)
{
  req.execution.header.stamp = ros::Time::now();
  pub_execution_info.publish(req.execution);
  return true;
}

bool AgentServer::check_if_collision(arm_operation::agent_abb_action::Request &req, 
                                     arm_operation::agent_abb_action::Response &res)
{
  const double gripper_width = 0.115/2;
  const double gripper_radius = 0.08/2;
  const double spear_radius = 0.09/2;
  const double spear_length = 0.19;
  double final_tcp_height = get_final_TCP_height(req.tool_id, req.position.z);
  double x_lower, camera_x, camera_y_lower, camera_y_upper, camera_z;
  double theta = M_PI/2-req.angle;
  bool will_collide = false;
  nh_.getParam("/combine_pc_node/x_lower", x_lower);
  nh_.getParam("/camera_x", camera_x);
  nh_.getParam("/camera_y_lower", camera_y_lower);
  nh_.getParam("/camera_y_upper", camera_y_upper);
  nh_.getParam("/camera_z", camera_z);
  x_lower -= 0.04f; // 4 cm to the wall of the tote
  // Spear
  if(final_tcp_height-spear_length<camera_z){
    if(in_range((camera_x-req.position.x)/spear_radius, -1, 1)){  // Valid theta
      double cos_  = (camera_x-req.position.x)/spear_radius,
             sin_1 = sqrt(1-cos_*cos_), sin_2 = -sin_1,
             intersect_1 = req.position.y + spear_radius*sin_1,
             intersect_2 = req.position.y + spear_radius*sin_2;
      will_collide = in_range(intersect_1, camera_y_lower, camera_y_upper) or 
                     in_range(intersect_2, camera_y_lower, camera_y_upper);
      if(will_collide) std::cout << "Spear collide\n";
    }
  }
  if(req.tool_id==1){
    // Lower body (parallel-jaw part)
    bool x_lower_collide = in_range((x_lower-req.position.x+gripper_width*cos(theta))/(2*gripper_width*cos(theta)), 0, 1);
    if(x_lower_collide) std::cout << "x_lower collide | lower body\n";
    will_collide = x_lower_collide or will_collide;
    // Upper body (circle part)
    x_lower_collide = (fabs(x_lower-req.position.x)<=gripper_radius);
    if(x_lower_collide) std::cout << "x_lower collide | upper body\n";
    will_collide = x_lower_collide or will_collide;
  }
  res.result = (will_collide?"true":"false");
  return true;
}
