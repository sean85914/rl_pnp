#ifndef HELPER_SERVICES_H_
#define HELPRR_SERVICES_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
// MSG
#include <geometry_msgs/Pose.h>
// SRV
// Standard
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
// Get target pose in robot arm frame
#include <grasp_suck/get_pose.h>
// Robot arm control
#include <arm_operation/joint_pose.h>
#include <arm_operation/target_pose.h>
// Suction gripper
#include <vacuum_conveyor_control/vacuum_control.h>

// Motion primitive
#define GRASP false
#define SUCK  true

const double OFFSET = -0.008f; // Lower 0.8 centimeter to make sure have contract with object
const double X_OFFSET = 0.02f; // Seems error from hand-eye calibration


class Helper_Services{
 private:
  // Variables
  bool define_home; // If define home joint
  bool define_place; // If define place joint
  bool last_motion; // Last motion primitive
  std::string cam_prefix; // Camera prefix
  std::string arm_prefix; // UR prefix
  std::vector<double> suction_tcp; // Suction TCP translation from end effector
  std::vector<double> gripper_tcp; // Gripper TCP translation from end effector
  std::vector<double> home_joint; 
  std::vector<double> place_joint;
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Service clients
  // Robot arm control
  ros::ServiceClient robot_arm_goto_joint;
  ros::ServiceClient robot_arm_goto_pose;
  // Vacuum control
  ros::ServiceClient vacuum_control;
  ros::ServiceClient pheumatic_control;
  // 2 finger gripper control
  ros::ServiceClient close_gripper;
  ros::ServiceClient open_gripper;
  ros::ServiceClient get_grasp_state;
  // Advertise services
  ros::ServiceServer service_home;
  ros::ServiceServer service_place;
  ros::ServiceServer service_goto_target;
  // End variables
  // Private methods
  bool go_home_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool go_place_service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool go_target_service_callback(grasp_suck::get_pose::Request &req, grasp_suck::get_pose::Response &res);
 public:
  // Constructor
  Helper_Services(ros::NodeHandle nh, ros::NodeHandle pnh);
};

#endif
