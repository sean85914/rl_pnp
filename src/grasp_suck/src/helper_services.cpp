#include "helper_services.h"

Helper_Services::Helper_Services(ros::NodeHandle nh, ros::NodeHandle pnh): 
 nh_(nh), pnh_(pnh), define_home(true), define_place(true)
{
  // Initialize vector size
  suction_tcp.resize(3); gripper_tcp.resize(3); home_joint.resize(6); place_joint.resize(6);
  // Get parameters
  if(!pnh.getParam("cam_prefix", cam_prefix)) cam_prefix = "camera1";
  if(!pnh.getParam("arn_prefix", arm_prefix)) arm_prefix = "";
  if(!pnh.getParam("/tcp_transformation_publisher/suction", suction_tcp)) {suction_tcp[0] = suction_tcp[1] = suction_tcp[2] = 0.0f;}
  if(!pnh.getParam("/tcp_transformation_publisher/gripper", gripper_tcp)) {gripper_tcp[0] = gripper_tcp[1] = gripper_tcp[2] = 0.0f;}
  if(!pnh.getParam("home_joint", home_joint)) {define_home = false; ROS_WARN("No predefined home joint received!");}
  if(!pnh.getParam("place_joint", place_joint)) {define_place = false; ROS_WARN("No predefined place joint received!");}
  // Show parameters
  ROS_INFO("--------------------------------------");
  ROS_INFO("Camera prefix: %s", cam_prefix.c_str());
  ROS_INFO("Arm prefix: %s", arm_prefix.c_str());
  ROS_INFO("Suction translation: %f %f %f", suction_tcp[0], suction_tcp[1], suction_tcp[2]);
  ROS_INFO("Gripper translation: %f %f %f", gripper_tcp[0], gripper_tcp[1], gripper_tcp[2]);
  if(define_home) ROS_INFO("UR5 home joints: %f %f %f %f %f %f", home_joint[0], home_joint[1], home_joint[2], home_joint[3], home_joint[4], home_joint[5]);
  if(define_place) ROS_INFO("UR5 place joints: %f %f %f %f %f %f", place_joint[0], place_joint[1], place_joint[2], place_joint[3], place_joint[4], place_joint[5]);
  ROS_INFO("--------------------------------------");
  // Publisher
  pub_marker = pnh_.advertise<visualization_msgs::Marker>("marker", 1);
  pub_text_marker = pnh_.advertise<visualization_msgs::Marker>("text_marker", 1);
  // Connect to service server
  // goto_joint_pose
  while(!ros::service::waitForService("/ur5_control_server/ur_control/goto_joint_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_joint_pose service...");}
  robot_arm_goto_joint = pnh.serviceClient<arm_operation::joint_pose>("/ur5_control_server/ur_control/goto_joint_pose");
  // goto_pose
  while(!ros::service::waitForService("/ur5_control_server/ur_control/goto_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_pose service...");}
  robot_arm_goto_pose = pnh.serviceClient<arm_operation::target_pose>("/ur5_control_server/ur_control/goto_pose");
  // vacuum_control
  while(!ros::service::waitForService("/arduino_control/vacuum_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to vacuum_control service...");}
  vacuum_control = pnh.serviceClient<vacuum_conveyor_control::vacuum_control>("/arduino_control/vacuum_control");
  // pheumatic_control
  while(!ros::service::waitForService("/arduino_control/pheumatic_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to pheumatic_control service...");}
  pheumatic_control = pnh.serviceClient<std_srvs::SetBool>("/arduino_control/pheumatic_control");
  // close_gripper
  while(!ros::service::waitForService("/robotiq_finger_control_node/close_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to close_gripper service...");}
  close_gripper = pnh.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/close_gripper");
  // open_gripper
  while(!ros::service::waitForService("/robotiq_finger_control_node/open_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to open_gripper service...");}
  open_gripper = pnh.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/open_gripper");
  // get_grasp_state
  /*while(!ros::service::waitForService("/robotiq_finger_control_node/get_grasp_state", ros::Duration(3.0))) {ROS_WARN("Try to connect to get_grasp_state service...");}
  get_grasp_state = pnh.serviceClient<std_srvs::Trigger>("/robotiq_finger_control_node/get_grasp_state");*/
  // set_prior
  while(!ros::service::waitForService("/get_reward/set_prior", ros::Duration(3.0))) {ROS_WARN("Try to connect to set_prior service...");}
  set_prior = pnh.serviceClient<std_srvs::Empty>("/get_reward/set_prior");
  // set_posterior
  while(!ros::service::waitForService("/get_reward/set_posterior", ros::Duration(3.0))) {ROS_WARN("Try to connect to set_posterior service...");}
  set_posterior = pnh.serviceClient<std_srvs::Empty>("/get_reward/set_posterior");
  // get_result
  while(!ros::service::waitForService("/get_reward/get_result", ros::Duration(3.0))) {ROS_WARN("Try to connect to get_result service...");}
  get_result = pnh.serviceClient<std_srvs::SetBool>("/get_reward/get_result");
  // Advertise service server
  // go home
  if(define_home) service_home = pnh.advertiseService("robot_go_home", 
                                                      &Helper_Services::go_home_service_callback, 
                                                      this);
  // go to placing
  if(define_place) service_place = pnh.advertiseService("robot_goto_place", 
                                                        &Helper_Services::go_place_service_callback, 
                                                        this);
  service_goto_target = pnh.advertiseService("goto_target", 
                                             &Helper_Services::go_target_service_callback, 
                                             this);
  ROS_INFO("\033[1;37mNode ready\033[0m");
  // TODO: add a service to check if suck succeed
}

/*
  _    _  ____  __  __ ______ 
 | |  | |/ __ \|  \/  |  ____|
 | |__| | |  | | \  / | |__   
 |  __  | |  | | |\/| |  __|  
 | |  | | |__| | |  | | |____ 
 |_|  |_|\____/|_|  |_|______|
                              
 */

bool Helper_Services::go_home_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  arm_operation::joint_pose myJointReq;
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = home_joint[i];
  ROS_INFO("UR5 goto home");
  robot_arm_goto_joint.call(myJointReq);
  std_srvs::Empty empty_req;
  set_posterior.call(empty_req);
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

bool Helper_Services::go_place_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  arm_operation::joint_pose myJointReq;
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = place_joint[i];
  ROS_INFO("UR5 goto place");
  robot_arm_goto_joint.call(myJointReq); ros::Duration(0.3).sleep();
  if(last_motion==false){ // Grasp
    std_srvs::Empty empty;
    open_gripper.call(empty);
  } else{ // Suck
    int tmp_counter = 0;
    do{ // Call serveral times to make sure the suction work properly
      ++tmp_counter; 
      vacuum_conveyor_control::vacuum_control vacuum_cmd;
      vacuum_cmd.request.command = 1; // Exhale
      vacuum_control.call(vacuum_cmd); 
      ros::Duration(0.2).sleep();
    }while(tmp_counter<=REPEAT_TIME-1);
    do{ // Call serveral times to make sure the suction work properly
      ++tmp_counter; 
      vacuum_conveyor_control::vacuum_control vacuum_cmd;
      vacuum_cmd.request.command = 0; // Normal
      vacuum_control.call(vacuum_cmd); 
      ros::Duration(0.2).sleep();
    }while(tmp_counter<=REPEAT_TIME);
    // Retract the pheumatic
    tmp_counter = 0;
    do{ // Call serveral times to make sure the suction work properly
      ++tmp_counter; 
      std_srvs::SetBool retract; 
      retract.request.data = false;
      pheumatic_control.call(retract);
      ros::Duration(0.2).sleep();
    }while(tmp_counter<=REPEAT_TIME-1);
  }
  // Then go home
  for(int i=0; i<6; ++i) myJointReq.request.joint[i] = home_joint[i];
  ROS_INFO("UR5 goto home"); 
  robot_arm_goto_joint.call(myJointReq);
  return true;
}

/*
   _____  ____    _______ ____    _______       _____   _____ ______ _______ 
  / ____|/ __ \  |__   __/ __ \  |__   __|/\   |  __ \ / ____|  ____|__   __|
 | |  __| |  | |    | | | |  | |    | |  /  \  | |__) | |  __| |__     | |   
 | | |_ | |  | |    | | | |  | |    | | / /\ \ |  _  /| | |_ |  __|    | |   
 | |__| | |__| |    | | | |__| |    | |/ ____ \| | \ \| |__| | |____   | |   
  \_____|\____/     |_|  \____/     |_/_/    \_\_|  \_\\_____|______|  |_|   
                                                                             
*/

bool Helper_Services::go_target_service_callback(
    grasp_suck::get_pose::Request &req, 
    grasp_suck::get_pose::Response &res)
{
  std_srvs::Empty empty_req;
  // Set prior depth image
  set_prior.call(empty_req);
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
  std::string des_frame = (arm_prefix==""?"base_link":arm_prefix+"_base_link");
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
  if(req.primmitive==GRASP and res.result_pose.position.z<0.21f) res.result_pose.position.z += 0.01f; // Low object, for instance, cuboid lying down
  if(req.primmitive==GRASP and res.result_pose.position.z>0.27f) res.result_pose.position.z += 0.01f; // Hight object, for instance, standed cylinder
  // Publish marker
  visualization_msgs::Marker marker, text_marker;
  marker.header.frame_id = ori_frame; text_marker.header.frame_id = des_frame;
  marker.action = visualization_msgs::Marker::ADD; text_marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE; text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.position = req.point_in_cam; 
  text_marker.pose.position.x = -0.7;
  marker.pose.orientation.w = 1.0; 
  marker.scale.x = marker.scale.y = marker.scale.z = 0.02; text_marker.scale.z = 0.06;
  if(req.primmitive==GRASP)  marker.color.b = marker.color.a = 1.0; // BLUE for grasping
  else marker.color.r = marker.color.a = 1.0; // RED for sucking
  text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0;
  int angle = req.yaw * 180 / M_PI;
  std::string marker_string = (req.primmitive==GRASP?"grasp, "+std::to_string(angle):"suck");
  text_marker.text = marker_string;
  pub_marker.publish(marker);
  pub_text_marker.publish(text_marker);
  arm_operation::target_pose myPoseReq;
  myPoseReq.request.target_pose = res.result_pose;
  myPoseReq.request.factor = 0.8f;
  ROS_INFO("\nUR5 goto target: \nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
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
      ros::Duration(0.2).sleep();
    }while(tmp_counter<=REPEAT_TIME);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "helper_service_node");
  ros::NodeHandle nh, pnh("~");
  Helper_Services foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
