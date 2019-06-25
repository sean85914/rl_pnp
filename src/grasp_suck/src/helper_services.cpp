#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
// SRV
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
// Get target pose in robot arm frame
#include <grasp_suck/get_pose.h>
// Robot arm high level control
#include <arm_operation/joint_pose.h>
#include <arm_operation/target_pose.h>
// Suction
#include <vacuum_conveyor_control/vacuum_control.h>

#define GRASP false
#define SUCK  true

bool define_home = true; 
bool define_place = true;
bool last_motion; // Last motion primmitive, 0 for grasp and 1 for suck
const double OFFSET = -0.008f; // Lower 0.8 centimeter to make sure have contact with object
const double X_OFFSET = 0.02f; // Seems error from calibration
std::string cam_prefix;
std::string ur_prefix;
std::vector<double> suction_tcp;
std::vector<double> gripper_tcp;
std::vector<double> ur3_home_joint;
std::vector<double> ur3_place_joint;
ros::ServiceClient robot_arm_goto_joint;
ros::ServiceClient robot_arm_goto_pose;
ros::ServiceClient vacuum_control;
ros::ServiceClient pheumatic_control;
ros::ServiceClient close_gripper;
ros::ServiceClient open_gripper;
ros::ServiceClient get_grasp_state;

/*
   _____  ____    _______ ____    _______       _____   _____ ______ _______ 
  / ____|/ __ \  |__   __/ __ \  |__   __|/\   |  __ \ / ____|  ____|__   __|
 | |  __| |  | |    | | | |  | |    | |  /  \  | |__) | |  __| |__     | |   
 | | |_ | |  | |    | | | |  | |    | | / /\ \ |  _  /| | |_ |  __|    | |   
 | |__| | |__| |    | | | |__| |    | |/ ____ \| | \ \| |__| | |____   | |   
  \_____|\____/     |_|  \____/     |_/_/    \_\_|  \_\\_____|______|  |_|   
                                                                             
*/                                                                             

bool service_callback(grasp_suck::get_pose::Request &req, grasp_suck::get_pose::Response &res){
  tf::TransformListener listener;
  ROS_INFO("\nReceive new request: ");
  if(req.primmitive==GRASP){ // Grasp
    last_motion = GRASP;
    printf("Motion primmitive: \033[1;32mgrasp\033[0m\n");
    printf("Position in camera frame: (%f, %f, %f)\n", req.point_in_cam.x, req.point_in_cam.y, req.point_in_cam.z);
    printf("Yaw: \n[radian] %f\n[Degree] %f\n", req.yaw, req.yaw*180.0/M_PI);
  } else{ // Suck
    last_motion = SUCK;
    // Extend the pheumatic first
    std_srvs::SetBool extend_cmd; extend_cmd.request.data = true;
    pheumatic_control.call(extend_cmd);
    printf("Motion primmitive: \033[1;32msuck\033[0m\n");
    printf("Position in camera frame: (%f, %f, %f)\n", req.point_in_cam.x, req.point_in_cam.y, req.point_in_cam.z);
  }
  std::string des_frame = (ur_prefix==""?"base_link":ur_prefix+"_base_link");
  std::string ori_frame = cam_prefix+"_color_optical_frame";
  tf::StampedTransform st;
  try{
    listener.waitForTransform(des_frame, ori_frame, ros::Time(0), ros::Duration(0.5));
    listener.lookupTransform(des_frame, ori_frame, ros::Time(0), st);
  }catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); return false;}
  tf::Transform tf(st.getRotation(), st.getOrigin());
  tf::Vector3 point_in_cam(req.point_in_cam.x, req.point_in_cam.y, req.point_in_cam.z),
              point_in_arm = tf*point_in_cam;
  tf::Matrix3x3 rotm(0.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 0.0f);
  tf::Quaternion q, q_gripper; 
  rotm.getRotation(q); 
  if(req.primmitive==GRASP){ // Grasp
    q_gripper.setRPY(req.yaw, 0.0, 0.0);
    q*=q_gripper;
  }
  res.result_pose.position.x = point_in_arm.getX();
  res.result_pose.position.y = point_in_arm.getY();
  res.result_pose.position.z = point_in_arm.getZ();
  res.result_pose.orientation.x = q.getX();
  res.result_pose.orientation.y = q.getY();
  res.result_pose.orientation.z = q.getZ();
  res.result_pose.orientation.w = q.getW();
  if(req.primmitive==GRASP){ // Grasp
    res.result_pose.position.z += gripper_tcp[0];
  } else{ // Suck
    res.result_pose.position.x += suction_tcp[2];
    res.result_pose.position.z += suction_tcp[0];
  }
  // Correction
  res.result_pose.position.x += X_OFFSET; 
  res.result_pose.position.z += OFFSET;
  arm_operation::target_pose myPoseReq;
  myPoseReq.request.target_pose = res.result_pose;
  myPoseReq.request.factor = 0.5f;
  ROS_INFO("\nUR3 goto target: \nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
            res.result_pose.position.x, res.result_pose.position.y, res.result_pose.position.z,
            res.result_pose.orientation.x, res.result_pose.orientation.y, res.result_pose.orientation.z, res.result_pose.orientation.w);
  robot_arm_goto_pose.call(myPoseReq);
  if(req.primmitive==GRASP){ // Grasp
    std_srvs::Empty empty;
    close_gripper.call(empty); // Close gripper
  } else{ // Suck
    int tmp_counter=0;
    do{ // Call twice to make sure the suction work properly
      ++tmp_counter; 
      vacuum_conveyor_control::vacuum_control vacuum_cmd;
      vacuum_cmd.request.command = 2; // Inhale
      vacuum_control.call(vacuum_cmd);
      ros::Duration(0.3).sleep();
    }while(tmp_counter<=1);
  }
  return true;
}

/*
  _    _  ____  __  __ ______ 
 | |  | |/ __ \|  \/  |  ____|
 | |__| | |  | | \  / | |__   
 |  __  | |  | | |\/| |  __|  
 | |  | | |__| | |  | | |____ 
 |_|  |_|\____/|_|  |_|______|
                              
 */

bool service_home_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  arm_operation::joint_pose myJointReq;
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = ur3_home_joint[i];
  ROS_INFO("UR3 goto home");
  robot_arm_goto_joint.call(myJointReq);
  if(last_motion==GRASP){
    std_srvs::Trigger req;
    get_grasp_state.call(req);
    ROS_INFO("Grasp: %s", req.response.success==true?"Success":"Fail");
  }else{ // Suck
    // TODO
  }
  return true;
}

/*
  _____  _               _____ ______ 
 |  __ \| |        /\   / ____|  ____|
 | |__) | |       /  \ | |    | |__   
 |  ___/| |      / /\ \| |    |  __|  
 | |    | |____ / ____ \ |____| |____ 
 |_|    |______/_/    \_\_____|______|
                                      
 */

bool service_place_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  arm_operation::joint_pose myJointReq;
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = ur3_place_joint[i];
  ROS_INFO("UR3 goto place");
  robot_arm_goto_joint.call(myJointReq); ros::Duration(0.3).sleep();
  if(last_motion==false){ // Grasp
    std_srvs::Empty empty;
    open_gripper.call(empty);
  } else{ // Suck
    int tmp_counter = 0;
    do{ // Call twice to make sure the suction work properly
      ++tmp_counter; 
      vacuum_conveyor_control::vacuum_control vacuum_cmd;
      vacuum_cmd.request.command = 0; // Exhale
      vacuum_control.call(vacuum_cmd); 
      ros::Duration(0.3).sleep();
    }while(tmp_counter<=1);
    // Retract the pheumatic
    tmp_counter = 0;
    do{ // Call twice to make sure the suction work properly
      ++tmp_counter; 
      std_srvs::SetBool retract; 
      retract.request.data = false;
      pheumatic_control.call(retract);
      ros::Duration(0.3).sleep();
    }while(tmp_counter<=1);
  }
  // Then go home
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = ur3_home_joint[i];
  ROS_INFO("UR3 goto home"); 
  robot_arm_goto_joint.call(myJointReq);
  return true;
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "helper_service_node");
  ros::NodeHandle nh, pnh("~");
  // Initialize vector size
  suction_tcp.resize(3); gripper_tcp.resize(3); ur3_home_joint.resize(6); ur3_place_joint.resize(6);
  // Get parameters
  if(!pnh.getParam("cam_prefix", cam_prefix)) cam_prefix = "camera1";
  if(!pnh.getParam("ur_prefix", ur_prefix)) ur_prefix = "";
  if(!pnh.getParam("/tcp_transformation_publisher/suction", suction_tcp)) {suction_tcp[0] = suction_tcp[1] = suction_tcp[2] = 0.0f;}
  if(!pnh.getParam("/tcp_transformation_publisher/gripper", gripper_tcp)) {gripper_tcp[0] = gripper_tcp[1] = gripper_tcp[2] = 0.0f;}
  if(!pnh.getParam("ur3_home_joint", ur3_home_joint)) {define_home = false; ROS_WARN("No predefined home joint received!");}
  if(!pnh.getParam("ur3_place_joint", ur3_place_joint)) {define_place = false; ROS_WARN("No predefined place joint received!");}
  // Show parameters
  ROS_INFO("--------------------------------------");
  ROS_INFO("Camera prefix: %s", cam_prefix.c_str());
  ROS_INFO("Arm prefix: %s", ur_prefix.c_str());
  ROS_INFO("Suction translation: %f %f %f", suction_tcp[0], suction_tcp[1], suction_tcp[2]);
  ROS_INFO("Gripper translation: %f %f %f", gripper_tcp[0], gripper_tcp[1], gripper_tcp[2]);
  if(define_home) ROS_INFO("UR3 home joints: %f %f %f %f %f %f", ur3_home_joint[0], ur3_home_joint[1], ur3_home_joint[2], ur3_home_joint[3], ur3_home_joint[4], ur3_home_joint[5]);
  if(define_place) ROS_INFO("UR3 place joints: %f %f %f %f %f %f", ur3_place_joint[0], ur3_place_joint[1], ur3_place_joint[2], ur3_place_joint[3], ur3_place_joint[4], ur3_place_joint[5]);
  ROS_INFO("--------------------------------------");
  // Advertise services
  ros::ServiceServer service_home;
  ros::ServiceServer service_place;
  ros::ServiceServer service_goto_target;
  // go home
  if(define_home){
    while(!ros::service::waitForService("/ur3_control_server/ur_control/goto_joint_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_joint_pose service...");}
    robot_arm_goto_joint = pnh.serviceClient<arm_operation::joint_pose>("/ur3_control_server/ur_control/goto_joint_pose");
    service_home = pnh.advertiseService("robot_go_home", service_home_cb);
  }
  // go to placing
  if(define_place){
    service_place = pnh.advertiseService("robot_goto_place", service_place_cb);
    while(!ros::service::waitForService("/arduino_control/vacuum_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to vacuum_control service...");}
    vacuum_control = pnh.serviceClient<vacuum_conveyor_control::vacuum_control>("/arduino_control/vacuum_control");
    while(!ros::service::waitForService("/arduino_control/pheumatic_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to pheumatic_control service...");}
    pheumatic_control = pnh.serviceClient<std_srvs::SetBool>("/arduino_control/pheumatic_control");
  }
  // go to target
  while(!ros::service::waitForService("/ur3_control_server/ur_control/goto_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_pose service...");}
  robot_arm_goto_pose = pnh.serviceClient<arm_operation::target_pose>("/ur3_control_server/ur_control/goto_pose");
  service_goto_target = pnh.advertiseService("goto_target", service_callback);
  // Gripper related service client
  // Close
  while(!ros::service::waitForService("/robotiq_finger_control_node/close_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to close_gripper service...");}
  close_gripper = pnh.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/close_gripper");
  // Open
  while(!ros::service::waitForService("/robotiq_finger_control_node/open_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to open_gripper service...");}
  open_gripper = pnh.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/open_gripper");
  // Get state
  while(!ros::service::waitForService("/robotiq_finger_control_node/get_grasp_state", ros::Duration(3.0))) {ROS_WARN("Try to connect to get_grasp_state service...");}
  get_grasp_state = pnh.serviceClient<std_srvs::Trigger>("/robotiq_finger_control_node/get_grasp_state");
  // Spin node until ROS shutdown
  ros::spin();
  return 0;
}
