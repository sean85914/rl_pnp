#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <grasp_suck/get_pose.h>

const double OFFSET = 0.01; // Lower 1 centimeter to make sure have contact with object
std::string cam_prefix;
std::string ur_prefix;
std::vector<double> suction_tcp;
std::vector<double> gripper_tcp;

bool service_callback(grasp_suck::get_pose::Request &req, grasp_suck::get_pose::Response &res){
  tf::TransformListener listener;
  ROS_INFO("\nReceive new request: ");
  if(req.primmitive==false){ // Grasp
    printf("Motion primmitive: \033[1;32mgrasp\033[0m\n");
    printf("Position in camera frame: (%f, %f, %f)\n", req.point_in_cam.x, req.point_in_cam.y, req.point_in_cam.z);
    printf("Yaw: \n[radian] %f\n[Degree] %f\n", req.yaw, req.yaw*180.0/M_PI);
  } else{ // Suck
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
  if(req.primmitive==false){ // Grasp
    q_gripper.setRPY(0.0, 0.0, req.yaw);
    q*=q_gripper;
  }
  res.result_pose.position.x = point_in_arm.getX();
  res.result_pose.position.y = point_in_arm.getY();
  res.result_pose.position.z = point_in_arm.getZ();
  res.result_pose.orientation.x = q.getX();
  res.result_pose.orientation.y = q.getY();
  res.result_pose.orientation.z = q.getZ();
  res.result_pose.orientation.w = q.getW();
  if(req.primmitive==false){ // Grasp
    res.result_pose.position.z += gripper_tcp[0];
  } else{ // Suck
    res.result_pose.position.x += suction_tcp[2];
    res.result_pose.position.z += suction_tcp[0];
  } res.result_pose.position.z += OFFSET;
  return true;
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "get_pose_node");
  ros::NodeHandle nh, pnh("~");
  suction_tcp.resize(3); gripper_tcp.resize(3);
  if(!pnh.getParam("cam_prefix", cam_prefix)) cam_prefix = "camera1";
  if(!pnh.getParam("ur_prefix", ur_prefix)) ur_prefix = "";
  if(!pnh.getParam("/tcp_transformation_publisher/suction", suction_tcp)) {suction_tcp[0] = suction_tcp[1] = suction_tcp[2] = 0.0f;}
  if(!pnh.getParam("/tcp_transformation_publisher/gripper", gripper_tcp)) {gripper_tcp[0] = gripper_tcp[1] = gripper_tcp[2] = 0.0f;}
  ROS_INFO("--------------------------------------");
  ROS_INFO("Camera prefix: %s", cam_prefix.c_str());
  ROS_INFO("Arm prefix: %s", ur_prefix.c_str());
  ROS_INFO("Suction translation: %f %f %f", suction_tcp[0], suction_tcp[1], suction_tcp[2]);
  ROS_INFO("Gripper translation: %f %f %f", gripper_tcp[0], gripper_tcp[1], gripper_tcp[2]);
  ROS_INFO("--------------------------------------");
  ros::ServiceServer service = pnh.advertiseService("get_pose_service", service_callback);
  ros::spin();
  return 0;
}
