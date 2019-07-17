/*
 *  Some useful services to control UR robot
 *  Subscribed topics:
 *    ~joint_states: joint state of universal robot
 *  Action interface:
 *    /follow_joint_trajectory
 *  Services:
 *    ~ur5_control/goto_pose: move robot TCP to given target pose in Cartesian space
 *    ~ur5_control/go_straight: move robot TCP from current pose to target pose straightly
 *    ~ur5_control/goto_joint_pose: move robot to given joint space
 *  Parameters:
 *    ~tool_length: length from ee_link to tcp_link
 *    ~sim: true if using simulation
 *    ~prefix: joint name prefix
 *    ~wrist1_upper_bound: wrist 1 upper bound
 *    ~wrist1_lower_bound: wrist 1 lower bound
 *    ~wrist2_upper_bound: wrist 2 upper bound
 *    ~wrist2_lower_bound: wrist 2 lower bound
 *    ~wrist3_upper_bound: wrist 3 upper bound
 *    ~wrist3_lower_bound: wrist 3 lower bound
 */
#ifndef UR_CONTROL_SERVER_H
#define UR_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// MSG
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
// SRV
#include <std_srvs/Empty.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_operation/target_pose.h>
#include <arm_operation/joint_pose.h>
#include <arm_operation/rotate_to_flip.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <Eigen/Dense>
#include <ur_kin.h>

#define deg2rad(x) (x*M_PI/180.0)
#define NUMBEROFPOINTS 10
// Make rotation more efficent
inline double makeMinorRotate(const double joint_now, const double joint_togo);

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm {
 private:
  // Varaibles
  int num_sols;
  double joint[6];
  double tool_length;
  double wrist1_upper_bound, wrist1_lower_bound;
  double wrist2_upper_bound, wrist2_lower_bound;
  double wrist3_upper_bound, wrist3_lower_bound;
  bool sim;
  bool wrist1_collision;
  bool wrist2_collision;
  bool wrist3_collision;
  std::string prefix;
  // ROS
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Subscriber
  ros::Subscriber sub_joint_state;
  // Services
  ros::ServiceServer goto_pose_srv;
  ros::ServiceServer go_straight_srv;
  ros::ServiceServer goto_joint_pose_srv;
  ros::ServiceServer fast_rotate_srv;
  ros::ServiceServer flip_srv;
  TrajClient *traj_client;
  control_msgs::FollowJointTrajectoryGoal goal; 
  control_msgs::FollowJointTrajectoryGoal path;
  // Private Functions
  /* 
   *  Convert input joint angle to branch [-pi, pi]
   *  Input:
   *    double angle: input joint angle
   *  Output:
   *    double: convert angle to branch [-pi, pi]
   */
  inline double validAngle(double angle);
  /*
   *  Subscriber callback, update joint values
   */
  void JointStateCallback(const sensor_msgs::JointState &msg);
  /*
   *  Convert pose to transformation matrix
   *  Input:
   *    geometry_msgs::Pose pose: input pose
   *    double *T: output placeholder
   *  Output: None
   */
  void PoseToDH(geometry_msgs::Pose pose, double *T);
  /*
   *  Perform inverse kinematic and return the optimized joints solution
   *  Input:
   *    geometry_msgs::Pose target_pose: target pose
   *    double *sol: output placeholder of size-6 double array
   *  Output:
   *    int: number of IK solutions
   */
  int PerformIK(geometry_msgs::Pose target_pose, double *sol);
  /*
   *  Perform inverse kinematic and return the optimized joints solution
   *  Will consider best wrist motion
   *  Input:
   *    geometry_msgs::Pose target_pose: target pose
   *    double *sol: output placeholder of size-6 double array
   *  Output:
   *    int: number of IK solutions
   */
  int PerformIKWristMotion(geometry_msgs::Pose target_pose, double *sol);
  /*
   *  Check if input wrist angle will self-collision
   *  Input:
   *    double &joint: wrist angle
   *    double upper: corresponding wrist upper bound
   *    double lower: corresponding wrist lower bound
   *  Output:
   *    bool: true if collision happened, false otherwise
   */
  inline bool wrist_check_bound(double &joint, double upper, double lower);
  /*
   *  Get current TCP pose
   *  Input: None
   *  Output:
   *    geometry_msgs::Pose: current TCP pose w.r.t. base link
   */
  geometry_msgs::Pose getCurrentTCPPose(void);
  /*
   *  Calculate execution time from start pose to target pose
   *  Input:
   *    const double *now: joint position array now
   *    const double *togo: target pose joint position array
   *    double factor: joint velocity factor, the larger the faster, default as 0.5
   *  Output:
   *    int: time to execute this trajectory
   */
  double calculate_time(const double *now, const double *togo, double factor=0.8);
  /*
   * Get traejctory execution state
   */
  inline actionlib::SimpleClientGoalState getState();
  /*
   *  Start trajectory with given goal
   *  Input:
   *    control_msgs::FollowJointTrajectoryGoal goal: target trajectory goal
   *  Output: None
   */
  inline void StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  /*
   *  Make robot arm go to target pose
   *  Input:
   *    geometry_msgs::Pose pose: target pose
   *    double factor: execution speed factor (the larger the faster)
   *  Output:
   *    control_msgs::FollowJointTrajectoryGoal
   */
  control_msgs::FollowJointTrajectoryGoal ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor);
 public:
   RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh);
   ~RobotArm();
   // Service server callback
   bool GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
   bool GoStraightLineService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
   bool GotoJointPoseService(arm_operation::joint_pose::Request &req, arm_operation::joint_pose::Response &res);
   bool FastRotateService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   bool FlipService(arm_operation::rotate_to_flip::Request &req,
                    arm_operation::rotate_to_flip::Response &res);
};

#endif
