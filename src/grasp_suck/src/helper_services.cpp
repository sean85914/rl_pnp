#include "helper_services.h"

Helper_Services::Helper_Services(ros::NodeHandle nh, ros::NodeHandle pnh): 
 nh_(nh), pnh_(pnh), define_home(true), define_place(true)
{
  // Initialize vector size
  suction_tcp.resize(3); gripper_tcp.resize(3); home_joint.resize(6); place_joint.resize(6); z_axis.resize(3);
  // Get parameters
  if(!pnh_.getParam("has_vacuum", has_vacuum)) has_vacuum = false;
  if(!pnh_.getParam("suck_offset", suck_offset)) suck_offset = -0.015f;
  if(!pnh_.getParam("grasp_offset", grasp_offset)) grasp_offset = 0.0f;
  if(!pnh_.getParam("/tcp_transformation_publisher/suction", suction_tcp)) {suction_tcp[0] = suction_tcp[1] = suction_tcp[2] = 0.0f;}
  if(!pnh_.getParam("/tcp_transformation_publisher/gripper", gripper_tcp)) {gripper_tcp[0] = gripper_tcp[1] = gripper_tcp[2] = 0.0f;}
  if(!pnh_.getParam("z_axis", z_axis)) {z_axis[0] = -1.0f; z_axis[1] = z_axis[2] = 0.0;}
  if(!pnh_.getParam("home_joint", home_joint)) {define_home = false; ROS_WARN("No predefined home joint received!");}
  if(!pnh_.getParam("place_joint", place_joint)) {define_place = false; ROS_WARN("No predefined place joint received!");}
  // Show parameters
  ROS_INFO("--------------------------------------");
  ROS_INFO("[%s] has_vacuum: %s", ros::this_node::getName().c_str(), (has_vacuum==true?"true":"false"));
  ROS_INFO("[%s] suck_offset: %f", ros::this_node::getName().c_str(), suck_offset);
  ROS_INFO("[%s] grasp_offset: %f", ros::this_node::getName().c_str(), grasp_offset);
  ROS_INFO("[%s] z_axis: %f %f %f", ros::this_node::getName().c_str(), z_axis[0], z_axis[1], z_axis[2]);
  ROS_INFO("[%s] Suction translation: %f %f %f", ros::this_node::getName().c_str(), suction_tcp[0], suction_tcp[1], suction_tcp[2]);
  ROS_INFO("[%s] Gripper translation: %f %f %f", ros::this_node::getName().c_str(), gripper_tcp[0], gripper_tcp[1], gripper_tcp[2]);
  if(define_home) ROS_INFO("[%s] UR5 home joints: %f %f %f %f %f %f", ros::this_node::getName().c_str(), home_joint[0], home_joint[1], home_joint[2], home_joint[3], home_joint[4], home_joint[5]);
  if(define_place) ROS_INFO("[%s] UR5 place joints: %f %f %f %f %f %f", ros::this_node::getName().c_str(), place_joint[0], place_joint[1], place_joint[2], place_joint[3], place_joint[4], place_joint[5]);
  ROS_INFO("--------------------------------------");
  // Publisher
  //pub_marker = pnh_.advertise<visualization_msgs::Marker>("marker", 1);
  //pub_text_marker = pnh_.advertise<visualization_msgs::Marker>("text_marker", 1);
  // Connect to service server
  // goto_joint_pose
  while(!ros::service::waitForService("/ur5_control_server/ur_control/goto_joint_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_joint_pose service...");}
  robot_arm_goto_joint = pnh_.serviceClient<arm_operation::joint_pose>("/ur5_control_server/ur_control/goto_joint_pose");
  // goto_pose
  while(!ros::service::waitForService("/ur5_control_server/ur_control/goto_pose", ros::Duration(3.0))) {ROS_WARN("Try to connect to goto_pose service...");}
  robot_arm_goto_pose = pnh_.serviceClient<arm_operation::target_pose>("/ur5_control_server/ur_control/goto_pose");
  // vacuum_control
  if(has_vacuum){
    while(!ros::service::waitForService("/arduino_control/vacuum_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to vacuum_control service...");}
    vacuum_control = pnh_.serviceClient<vacuum_conveyor_control::vacuum_control>("/arduino_control/vacuum_control");
  }
  // pheumatic_control
  if(has_vacuum){
    while(!ros::service::waitForService("/arduino_control/pheumatic_control", ros::Duration(3.0))) {ROS_WARN("Try to connect to pheumatic_control service...");}
    pheumatic_control = pnh_.serviceClient<std_srvs::SetBool>("/arduino_control/pheumatic_control");
  }
  // close_gripper
  while(!ros::service::waitForService("/robotiq_finger_control_node/close_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to close_gripper service...");}
  close_gripper = pnh_.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/close_gripper");
  // open_gripper
  while(!ros::service::waitForService("/robotiq_finger_control_node/open_gripper", ros::Duration(3.0))) {ROS_WARN("Try to connect to open_gripper service...");}
  open_gripper = pnh_.serviceClient<std_srvs::Empty>("/robotiq_finger_control_node/open_gripper");
  // Advertise service server
  // go home
  assert(define_home);
  service_home = pnh_.advertiseService("robot_go_home", 
                                      &Helper_Services::go_home_service_callback, 
                                      this);
  // go to placing
  assert(define_place);
  service_place = pnh_.advertiseService("robot_goto_place", 
                                       &Helper_Services::go_place_service_callback, 
                                       this);
  service_goto_target = pnh_.advertiseService("goto_target", 
                                             &Helper_Services::go_target_service_callback, 
                                             this);
  // Timer
  param_timer = pnh_.createTimer(ros::Duration(1.0), &Helper_Services::timer_callback, this);
  ROS_INFO("\033[1;37mNode ready\033[0m");
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
  myJointReq.request.joints.resize(1);
  for(int i=0; i<6; ++i) myJointReq.request.joints[0].joint_value[i] = home_joint[i];
  ROS_INFO("UR5 goto home");
  robot_arm_goto_joint.call(myJointReq);
  std_srvs::Empty empty_req;
  //set_posterior.call(empty_req);
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
  myJointReq.request.joints.resize(1);
  for(int i=0; i<6; ++i) myJointReq.request.joints[0].joint_value[i] = place_joint[i];
  ROS_INFO("UR5 goto place");
  robot_arm_goto_joint.call(myJointReq); ros::Duration(0.3).sleep();
  if(last_motion==false){ // Grasp
    std_srvs::Empty empty;
    open_gripper.call(empty);
    ros::Duration(0.3).sleep();
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
  for(int i=0; i<6; ++i) myJointReq.request.joints[0].joint_value[i] = home_joint[i];
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
  tf::TransformListener listener;
  ROS_INFO("\nReceive new request: ");
  if(req.primitive==GRASP){ // Grasp
    last_motion = GRASP;
    printf("Motion primitive: \033[1;32mgrasp\033[0m\n");
    printf("Position in hand frame: (%f, %f, %f)\n", req.point_in_hand.x, req.point_in_hand.y, req.point_in_hand.z);
    printf("Yaw: \n[radian] %f\n[Degree] %f\n", req.yaw, req.yaw*180.0/M_PI);
  } else{ // Suck
    last_motion = SUCK;
    // Extend the pheumatic first
    std_srvs::SetBool extend_cmd; extend_cmd.request.data = true;
    pheumatic_control.call(extend_cmd);
    printf("Motion primitive: \033[1;32msuck\033[0m\n");
    printf("Position in hand frame: (%f, %f, %f)\n", req.point_in_hand.x, req.point_in_hand.y, req.point_in_hand.z);
    printf("Surface normal: (%f, %f, %f)\n", req.x_axis.x, req.x_axis.y, req.x_axis.z);
  }
  tf::Vector3 point_in_arm(req.point_in_hand.x, req.point_in_hand.y, req.point_in_hand.z);
  tf::Quaternion q, q_gripper; 
  if(req.primitive==GRASP){ // Grasp
    tf::Vector3 x_axis_tf(0.0f, 0.0f, -1.0f),
                z_axis_tf(z_axis[0], z_axis[1], z_axis[2]),
                y_axis_tf = z_axis_tf.cross(x_axis_tf);
    tf::Matrix3x3 rotm(x_axis_tf.x(), y_axis_tf.x(), z_axis_tf.x(),
                       x_axis_tf.y(), y_axis_tf.y(), z_axis_tf.y(),
                       x_axis_tf.z(), y_axis_tf.z(), z_axis_tf.z());
    rotm.getRotation(q); 
    q_gripper.setRPY(req.yaw, 0.0, 0.0);
    q*=q_gripper;
  } else{ // SUCK
    tf::Vector3 x_axis_tf(req.x_axis.x, req.x_axis.y, req.x_axis.z), 
                fake_z(z_axis[0], z_axis[1], z_axis[2]);
    if(x_axis_tf.getX()==0.0f and x_axis_tf.getY()==0.0f and x_axis_tf.getZ()==0.0f){
      ROS_WARN("Incorrect surface normal given, using default perpendicular one...");
      x_axis_tf = tf::Vector3(0.0f, 0.0f, -1.0f);
    }
    tf::Vector3 y_axis_tf = fake_z.cross(x_axis_tf), z_axis_tf = x_axis_tf.cross(y_axis_tf);
    tf::Matrix3x3 rotm(x_axis_tf.x(), y_axis_tf.x(), z_axis_tf.x(),
                       x_axis_tf.y(), y_axis_tf.y(), z_axis_tf.y(),
                       x_axis_tf.z(), y_axis_tf.z(), z_axis_tf.z());
    rotm.getRotation(q); 
  }
  res.result_pose.orientation.x = q.getX();
  res.result_pose.orientation.y = q.getY();
  res.result_pose.orientation.z = q.getZ();
  res.result_pose.orientation.w = q.getW();
  if(req.primitive==GRASP){ // Grasp
    res.result_pose.position.x = point_in_arm.getX();
    res.result_pose.position.y = point_in_arm.getY();
    res.result_pose.position.z = point_in_arm.getZ();
    res.result_pose.position.z += gripper_tcp[0];
  } else{ // Suck
    // Perpendicular suck
    //res.result_pose.position.x += suction_tcp[2];
    //res.result_pose.position.z += suction_tcp[0];
    // Consider surface normal
    tf::Transform homo_mat(q, point_in_arm);
    tf::Vector3 suction_cup_tcp(-suction_tcp[0], 0.0f, -suction_tcp[2]),
                final_position = homo_mat*suction_cup_tcp;
    res.result_pose.position.x = final_position.getX();
    res.result_pose.position.y = final_position.getY();
    res.result_pose.position.z = final_position.getZ();
  }
  // First waypoint, height 20 cm
  tf::Matrix3x3 rotm(tf::Quaternion(res.result_pose.orientation.x, res.result_pose.orientation.y, res.result_pose.orientation.z, res.result_pose.orientation.w));
  tf::Vector3 first_wp(-0.2f, 0.0f, 0.0f), first_wp_offset = rotm*first_wp;
  arm_operation::target_pose myPoseReq;
  myPoseReq.request.target_pose = res.result_pose;
  myPoseReq.request.target_pose.position.x = res.result_pose.position.x+first_wp_offset.getX();
  myPoseReq.request.target_pose.position.y = res.result_pose.position.y+first_wp_offset.getY();
  myPoseReq.request.target_pose.position.z = res.result_pose.position.z+first_wp_offset.getZ();
  myPoseReq.request.factor = 0.5f;
  ROS_INFO("\nUR5 goto temp waypoint: \nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
            myPoseReq.request.target_pose.position.x, myPoseReq.request.target_pose.position.y, myPoseReq.request.target_pose.position.z,
            res.result_pose.orientation.x, res.result_pose.orientation.y, res.result_pose.orientation.z, res.result_pose.orientation.w);
  robot_arm_goto_pose.call(myPoseReq);
  // Correction
  if(req.primitive==SUCK)  res.result_pose.position.z += suck_offset;
  if(req.primitive==GRASP) res.result_pose.position.z += grasp_offset;
  if(req.primitive==SUCK and res.result_pose.position.z>=0.04f) res.result_pose.position.z += 0.005f;
  myPoseReq.request.target_pose = res.result_pose;
  ROS_INFO("\nUR5 goto target: \nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
            myPoseReq.request.target_pose.position.x, myPoseReq.request.target_pose.position.y, myPoseReq.request.target_pose.position.z,
            res.result_pose.orientation.x, res.result_pose.orientation.y, res.result_pose.orientation.z, res.result_pose.orientation.w);
  robot_arm_goto_pose.call(myPoseReq);
  if(req.primitive==GRASP){ // Grasp
    std_srvs::Empty empty;
    close_gripper.call(empty); // Close gripper
    ros::Duration(0.5).sleep();
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

void Helper_Services::timer_callback(const ros::TimerEvent& event){
  double suck_tmp, grasp_tmp;
  pnh_.getParam("suck_offset", suck_tmp); pnh_.getParam("grasp_offset", grasp_tmp);
  if(suck_tmp!=suck_offset){
    ROS_INFO("[%s] suck_offset changes from %f to %f", ros::this_node::getName().c_str(), suck_offset, suck_tmp);
    suck_offset = suck_tmp;
  }
  if(grasp_tmp!=grasp_offset){
    ROS_INFO("[%s] grasp_offset changes from %f to %f", ros::this_node::getName().c_str(), grasp_offset, grasp_tmp);
    grasp_offset = grasp_tmp;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "helper_service_node");
  ros::NodeHandle nh, pnh("~");
  Helper_Services foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
