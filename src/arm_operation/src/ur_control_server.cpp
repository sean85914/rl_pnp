#include "ur_control_server.h"

// Public functions
RobotArm::RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), num_sols(1){
  // Subscriber
  sub_joint_state = pnh_.subscribe("joint_states", 1, &RobotArm::JointStateCallback, this);
  // Service server
  goto_pose_srv = pnh_.advertiseService("ur_control/goto_pose", &RobotArm::GotoPoseService, this);
  go_straight_srv = pnh_.advertiseService("ur_control/go_straight", &RobotArm::GoStraightLineService, this);
  go_parabolic_srv = pnh_.advertiseService("ur_control/go_parabolic", &RobotArm::GoParabolicService, this);
  goto_joint_pose_srv = pnh_.advertiseService("ur_control/goto_joint_pose", &RobotArm::GotoJointPoseService, this);
  fast_rotate_srv = pnh_.advertiseService("ur_control/fast_rotate", &RobotArm::FastRotateService, this);
  // Parameters
  if(!pnh.getParam("tool_length", tool_length)) tool_length = 0.18;
  if(!pnh.getParam("prefix", prefix)) prefix="";
  if(!pnh.getParam("sim", sim)) sim = false;
  // Wrist1 default bound [-240, -30]
  if(!pnh.getParam("wrist1_upper_bound", wrist1_upper_bound)) wrist1_upper_bound = deg2rad(-30);
  if(!pnh.getParam("wrist1_lower_bound", wrist1_lower_bound)) wrist1_lower_bound = deg2rad(-240);
  // Wrist2 default bound [-pi, 0]
  if(!pnh.getParam("wrist2_upper_bound", wrist2_upper_bound)) wrist2_upper_bound = 0;
  if(!pnh.getParam("wrist2_lower_bound", wrist2_lower_bound)) wrist2_lower_bound = -M_PI;
  // Wrist3 default bound [-220, 5]
  if(!pnh.getParam("wrist3_upper_bound", wrist3_upper_bound)) wrist3_upper_bound = deg2rad(5);
  if(!pnh.getParam("wrist3_lower_bound", wrist3_lower_bound)) wrist3_lower_bound = deg2rad(-220);
  // Show parameter information
  ROS_INFO("*********************************************************************************");
  ROS_INFO("[%s] Tool length: %f", ros::this_node::getName().c_str(), tool_length);
  ROS_INFO("[%s] Prefix: %s", ros::this_node::getName().c_str(), prefix.c_str());
  ROS_INFO("[%s] Sim: %s", ros::this_node::getName().c_str(), (sim==true?"True":"False"));
  ROS_INFO("[%s] Wrist 1 upper bound: %f, lower bound: %f", ros::this_node::getName().c_str(), wrist1_upper_bound, wrist1_lower_bound);
  ROS_INFO("[%s] Wrist 2 upper bound: %f, lower bound: %f", ros::this_node::getName().c_str(), wrist2_upper_bound, wrist2_lower_bound);
  ROS_INFO("[%s] Wrist 3 upper bound: %f, lower bound: %f", ros::this_node::getName().c_str(), wrist3_upper_bound, wrist3_lower_bound);
  ROS_INFO("*********************************************************************************");
  // Tell the action client that we want to spin a thread by default
  if(!sim)
    traj_client = new TrajClient("/follow_joint_trajectory", true);
  else
    traj_client = new TrajClient("/arm_controller/follow_joint_trajectory", true);
  // Wait for action server to come up
  while (!traj_client->waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the joint_trajectory_action server");
  ROS_INFO("[%s] Action server connected!", ros::this_node::getName().c_str());
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  t.joint_names.resize(6);                   l.joint_names.resize(6);
  t.joint_names[0] = prefix+"shoulder_pan_joint";   l.joint_names[0] = prefix+"shoulder_pan_joint";
  t.joint_names[1] = prefix+"shoulder_lift_joint";  l.joint_names[1] = prefix+"shoulder_lift_joint";
  t.joint_names[2] = prefix+"elbow_joint";          l.joint_names[2] = prefix+"elbow_joint";
  t.joint_names[3] = prefix+"wrist_1_joint";        l.joint_names[3] = prefix+"wrist_1_joint";
  t.joint_names[4] = prefix+"wrist_2_joint";        l.joint_names[4] = prefix+"wrist_2_joint";
  t.joint_names[5] = prefix+"wrist_3_joint";        l.joint_names[5] = prefix+"wrist_3_joint";
  t.points.resize(2);
  for(int i=0; i<2; ++i){
    t.points[i].positions.resize(6);
    t.points[i].velocities.resize(6);
  }
  l.points.resize(NUMBEROFPOINTS+1);
  for(int i=0; i<NUMBEROFPOINTS+1; ++i) {
    l.points[i].positions.resize(6);
    l.points[i].velocities.resize(6);
  }
}

RobotArm::~RobotArm(){
  delete traj_client;
}

bool RobotArm::GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  ROS_INFO("[%s] Receive new pose goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                               req.target_pose.position.x,
                                                               req.target_pose.position.y,
                                                               req.target_pose.position.z,
                                                               req.target_pose.orientation.x,
                                                               req.target_pose.orientation.y, 
                                                               req.target_pose.orientation.z, 
                                                               req.target_pose.orientation.w);
  StartTrajectory(ArmToDesiredPoseTrajectory(req.target_pose, req.factor));
  if(num_sols == 0) res.plan_result = "fail_to_find_solution";
  res.plan_result = "find_one_feasible_solution";
  return true;
}

bool RobotArm::GoStraightLineService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  ROS_INFO("[%s] Receive new straight line goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                                        req.target_pose.position.x, 
                                                                        req.target_pose.position.y,
                                                                        req.target_pose.position.z,
                                                                        req.target_pose.orientation.x,
                                                                        req.target_pose.orientation.y,
                                                                        req.target_pose.orientation.z,
                                                                        req.target_pose.orientation.w);
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  geometry_msgs::Pose pose_now = getCurrentTCPPose();
  double waypoint_sol_[NUMBEROFPOINTS * 6] = {0}, temp[6] = {0};
  tf::Quaternion q(pose_now.orientation.x, 
                   pose_now.orientation.y,
                   pose_now.orientation.z,
                   pose_now.orientation.w), 
                 q_inter_,
                 q_goal_(req.target_pose.orientation.x, 
                         req.target_pose.orientation.y,
                         req.target_pose.orientation.z,
                         req.target_pose.orientation.w);
  for(int i=0; i<NUMBEROFPOINTS; ++i) {
    geometry_msgs::Pose waypoint;
    // Interpolated points
    waypoint.position.x = pose_now.position.x + (req.target_pose.position.x - pose_now.position.x) * (i+1) / double(NUMBEROFPOINTS);
    waypoint.position.y = pose_now.position.y + (req.target_pose.position.y - pose_now.position.y) * (i+1) / double(NUMBEROFPOINTS);
    waypoint.position.z = pose_now.position.z + (req.target_pose.position.z - pose_now.position.z) * (i+1) / double(NUMBEROFPOINTS);
    q_inter_ = q.slerp(q_goal_, (i+1) / double(NUMBEROFPOINTS));
    waypoint.orientation.x = q_inter_.getX();
    waypoint.orientation.y = q_inter_.getY();
    waypoint.orientation.z = q_inter_.getZ();
    waypoint.orientation.w = q_inter_.getW();
    PerformIK(waypoint, temp);
    if(num_sols == 0) {
      ROS_ERROR("waypoint index: %d fail to find IK solution", i);
      res.plan_result = "fail_to_find_solution";
      return true;
    }
    for(int j=0; j<6; ++j) {waypoint_sol_[i*6 + j] = temp[j];}
  }
  double total_time = calculate_time(joint, temp, req.factor); // Total time cost from initial to goal
  ROS_INFO("[%s] Execution time: %.f seconds", ros::this_node::getName().c_str(), total_time);
  // Least Square to solve joints angular velocities
  // Assume: q(t) = a*t^2 + b*t + c
  // Then:   w(t) = 2a*t + b
  Eigen::MatrixXd A(NUMBEROFPOINTS+1, 3);
  Eigen::MatrixXd b(NUMBEROFPOINTS+1, 6);
  // Build matrix A and b
  for (int i = 0; i < NUMBEROFPOINTS; ++i) {
    double t = total_time * (i+1) / double(NUMBEROFPOINTS);
    A(i+1, 0) = t*t; A(i+1, 1) = t; A(i+1, 2) = 1;
    for (int j=0; j<6; ++j) {
      b(i+1, j) = waypoint_sol_[i*6 + j];
    }
  }
  A(0, 0) = 0; A(0, 1) = 0; A(0, 2) = 1;
  for(int j = 0; j<6; ++j) {b(0, j)=joint[j];}
  Eigen::MatrixXd x(3, 6); x = (A.transpose()*A).inverse()*A.transpose()*b;
  for (int i=0; i<NUMBEROFPOINTS; ++i) {
    double temp[6]={0};
    for (int j=0; j<6; ++j){
      l.points[i+1].positions[j] = waypoint_sol_[i*6 + j];
      l.points[i+1].velocities[j] = 2* x(0, j) * total_time * (i+1) / double(NUMBEROFPOINTS) + x(1, j);
      l.points[i+1].time_from_start = ros::Duration(total_time * (i+1) / double(NUMBEROFPOINTS));
    }
  }
  for (int i=0; i<6; ++i) {
    l.points[0].positions[i] = joint[i];
    l.points[0].velocities[i] = 0;
    l.points[NUMBEROFPOINTS].velocities[i] = 0; 
    l.points[0].time_from_start = ros::Duration(0);
  }
  StartTrajectory(path);
  res.plan_result = "find_one_feasible_solution";
  return true;
}

bool RobotArm::GoParabolicService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  ROS_INFO("[%s] Receive new parabola path: %f %f %f", ros::this_node::getName().c_str(),
                                                       req.target_pose.position.x, 
                                                       req.target_pose.position.y,
                                                       req.target_pose.position.z);
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  double waypoint_sol_[NUMBEROFPOINTS * 6] = {0}, temp[6] = {0};
  for(int i=0; i<NUMBEROFPOINTS; ++i) {
    geometry_msgs::Pose waypoint;
    // Interpolated points
    waypoint.position.x = req.target_pose.position.x;
    waypoint.position.y = req.target_pose.position.y - move_y * i / double(NUMBEROFPOINTS);
    waypoint.position.z = parabola_a*pow(waypoint.position.y, 2)+parabola_b*waypoint.position.y+parabola_c;
    //ROS_INFO("%f %f %f", waypoint.position.x, waypoint.position.y, waypoint.position.z);
    waypoint.orientation.x = 0.568; //-0.5;
    waypoint.orientation.y = -0.524; //0.5;
    waypoint.orientation.z = -0.428; //0.5;
    waypoint.orientation.w = -0.469; //0.5;
    PerformIK(waypoint, temp);
    if(num_sols == 0) {
      ROS_ERROR("waypoint index: %d fail to find IK solution", i);
      res.plan_result = "fail_to_find_solution";
      return true;
    }
    for(int j=0; j<6; ++j) {waypoint_sol_[i*6 + j] = temp[j];}
  }
  // Least Square to solve joints angular velocities
  // Assume: q(t) = a*t^2 + b*t + c
  // Then:   w(t) = 2a*t + b
  Eigen::MatrixXd A(NUMBEROFPOINTS+1, 3);
  Eigen::MatrixXd b(NUMBEROFPOINTS+1, 6);
  // Build matrix A and b
  for (int i = 0; i < NUMBEROFPOINTS; ++i) {
    double t = parabola_time * (i+1) / double(NUMBEROFPOINTS);
    A(i+1, 0) = t*t; A(i+1, 1) = t; A(i+1, 2) = 1;
    for (int j=0; j<6; ++j) {
      b(i+1, j) = waypoint_sol_[i*6 + j];
    }
  }
  A(0, 0) = 0; A(0, 1) = 0; A(0, 2) = 1;
  for(int j = 0; j<6; ++j) {b(0, j)=joint[j];}
  Eigen::MatrixXd x(3, 6); x = (A.transpose()*A).inverse()*A.transpose()*b;
  for (int i=0; i<NUMBEROFPOINTS; ++i) {
    double temp[6]={0};
    for (int j=0; j<6; ++j){
      l.points[i+1].positions[j] = waypoint_sol_[i*6 + j];
      l.points[i+1].velocities[j] = 2* x(0, j) * parabola_time * (i+1) / double(NUMBEROFPOINTS) + x(1, j);
      l.points[i+1].time_from_start = ros::Duration(parabola_time * (i+1) / double(NUMBEROFPOINTS));
    }
  }
  for (int i=0; i<6; ++i) {
    l.points[0].positions[i] = joint[i];
    l.points[0].velocities[i] = 0;
    l.points[NUMBEROFPOINTS].velocities[i] = 0; 
    l.points[0].time_from_start = ros::Duration(0);
  }
  /*
  for (int i=0; i<NUMBEROFPOINTS; ++i) {
    double temp[6]={0};
    for (int j=0; j<6; ++j){
      l.points[i+1].positions[j] = waypoint_sol_[i*6 + j];
      l.points[i+1].velocities[j] = 0.1; // XXX
      l.points[i+1].time_from_start = ros::Duration(parabola_time * (i+1) / double(NUMBEROFPOINTS));
    }
  }
  for (int i=0; i<6; ++i) {
    l.points[0].positions[i] = joint[i];
    l.points[0].velocities[i] = 0;
    l.points[NUMBEROFPOINTS].velocities[i] = 0; 
    l.points[0].time_from_start = ros::Duration(0);
  }
  */
  StartTrajectory(path);
  res.plan_result = "find_one_feasible_solution";
  return true;
}

bool RobotArm::GotoJointPoseService(arm_operation::joint_pose::Request  &req, arm_operation::joint_pose::Response &res){
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  ROS_INFO("[%s] Receive new joint pose request: %f %f %f %f %f %f", 
            ros::this_node::getName().c_str(), req.joint[0], req.joint[1], req.joint[2], req.joint[3], req.joint[4], req.joint[5]);
  for (int i = 0; i < 6; ++i) {
    t.points[0].positions[i] = joint[i];
    t.points[1].positions[i] = req.joint[i];
    t.points[0].velocities[i] = 0;
    t.points[1].velocities[i] = 0;
  }
  double togo[6]; for(int i=0; i<6; ++i) togo[i] = req.joint[i];
  t.points[0].time_from_start = ros::Duration(0);
  t.points[1].time_from_start = ros::Duration(calculate_time(joint, togo));
  ROS_INFO("[%s] Execution time: %f seconds", ros::this_node::getName().c_str(), calculate_time(joint, togo));
  StartTrajectory(goal);
  res.plan_result = "Success";
  return true;
}

bool RobotArm::FastRotateService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("Receive fast rotate service call.");
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  for(int i=0; i<6; ++i){
    t.points[0].positions[i] = joint[i];
    t.points[1].positions[i] = joint[i];
    t.points[0].velocities[i] = 0.0;
    t.points[1].velocities[i] = 0.0;
  }
  t.points[1].positions[5] += M_PI/2; //if(t.points[1].positions[5] >= 2*M_PI) t.points[1].positions[5] -= 2*M_PI;
  t.points[0].time_from_start = ros::Duration(0);
  t.points[1].time_from_start = ros::Duration(0.5);
  StartTrajectory(goal);
  return true;
}

// Private functions

double RobotArm::validAngle(double angle){
  if(abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
  if(angle > M_PI) angle -= 2*M_PI;
  else if(angle < -M_PI) angle += 2*M_PI;
  return angle;
}

void RobotArm::JointStateCallback(const sensor_msgs::JointState &msg){
  if(!sim){
    for(int i=0; i<6; ++i){
      joint[i] = msg.position[i];
    }
  }
  else{
    joint[0] = msg.position[2]; // shoulder_pan_joint
    joint[1] = msg.position[1]; // shoulder_lift_joint
    joint[2] = msg.position[0]; // elbow_joint
    for(int i=3; i<6; ++i) joint[i] = msg.position[i];
  }
}

void RobotArm::PoseToDH(geometry_msgs::Pose pose, double *T){
  double roll = 0, pitch = 0, yaw = 0;
  geometry_msgs::Point &p = pose.position;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double sinr = sin(roll), cosr = cos(roll);
  double sinp = sin(pitch), cosp = cos(pitch);
  double siny = sin(yaw), cosy = cos(yaw);
  // DH matrix, ZYX convention
  T[0] = cosy*cosp;
  T[1] = cosy*sinp*sinr - cosr*siny;
  T[2] = siny*sinr + cosy*cosr*sinp;
  T[3] = p.x;
  T[4] = cosp*siny;
  T[5] = cosy*cosr + siny*sinp*sinr;
  T[6] = cosr*siny*sinp - cosy*sinr;
  T[7] = p.y;
  T[8] = -sinp;
  T[9] = cosp*sinr;
  T[10] = cosp*cosr;
  T[11] = p.z;
  T[15] = 1;
}

int RobotArm::PerformIK(geometry_msgs::Pose target_pose, double *sol){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6], min = 1e6, dist = 0;
  int sols = ur_kinematics::inverse(T, q_sols), index = -1;
  for (int i = 0; i < sols; ++i) {
    // Preprocess joint angle to -pi ~ pi
    for (int j = 0; j < 6; ++j) {
      q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
    } 
    // Convert wrist joints to available range
    wrist1_collision = wrist_check_bound(q_sols[i*6+3], wrist1_upper_bound, wrist1_lower_bound); 
    wrist2_collision = wrist_check_bound(q_sols[i*6+4], wrist2_upper_bound, wrist2_lower_bound); 
    wrist3_collision = wrist_check_bound(q_sols[i*6+5], wrist3_upper_bound, wrist3_lower_bound);
    for (int j = 0; j < 6; ++j) {
      if(j<3) dist += pow((q_sols[i*6 + j] - joint[j])*1.5, 2); // For joint 1, 2, and 3, multiply by 1.5
      else dist += pow((q_sols[i*6 + j] - joint[j])*0.5, 2); // For joint 4, 5 and 6, multiply by 0.5
    }
    // Find solution with minimum joint angle difference
    if(min>dist && !wrist1_collision && !wrist2_collision && !wrist3_collision){
      min = dist;
      index = i;
    } dist = 0; 
  } if(index == -1) return 0; // All solutions will self-collision
  for(int i=0; i<6; ++i) sol[i] = q_sols[index*6+i];
  return (num_sols = sols);
}

bool RobotArm::wrist_check_bound(double &joint, double upper, double lower){
  if(joint>upper) joint-=2*M_PI;
  else if(joint<lower) joint+=2*M_PI;
  if(joint>lower and joint<upper) return false;
  return true;
}

geometry_msgs::Pose RobotArm::getCurrentTCPPose(void){
  geometry_msgs::Pose pose_now;
  ros::spinOnce(); // Update joint state
  double T[16] = {0};
  ur_kinematics::forward(joint, T);
  tf::Quaternion q;
  tf::Matrix3x3 mat;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat[i][j] = T[i*4 + j];
    }
  }
  mat.getRotation(q);
  // ee_link to tcp_link
  pose_now.position.x = T[3] + T[0] * tool_length;
  pose_now.position.y = T[7] + T[4] * tool_length;
  pose_now.position.z = T[11] + T[8] * tool_length;
  pose_now.orientation.x = q.getX();
  pose_now.orientation.y = q.getY();
  pose_now.orientation.z = q.getZ();
  pose_now.orientation.w = q.getW();
  return pose_now;
}

double RobotArm::calculate_time(const double *now, const double *togo, double factor){
  double time;
  if(factor == 0.0) {ROS_WARN("Invalid factor, set to 0.5"); factor = 0.5;}
  double dist = 0;
  for(int i=0; i<6; ++i){
    dist += pow(now[i] - togo[i], 2);
  }
  if(sqrt(dist)/factor<0.5) time = 0.5;
  else time = ceil(sqrt(dist)/factor);
  return (time);
}

actionlib::SimpleClientGoalState RobotArm::getState() {
  return traj_client->getState();
}

void RobotArm::StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal){
  goal.trajectory.header.stamp = ros::Time::now();
  traj_client->sendGoal(goal);
  // Wait for ur to finish joint trajectory
  while(!traj_client->getState().isDone() && ros::ok()) usleep(100000);
}

control_msgs::FollowJointTrajectoryGoal RobotArm::ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor){
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  // Get closest joint space solution
  double sol[6] = {0}; ROS_INFO("[%s] Joints to go solve from IK: ", ros::this_node::getName().c_str());
  // If not solutions, assign angle now to sol
  if(!PerformIK(pose, sol)) {
    ROS_WARN("Cannot find IK solution!");
    for (int i = 0; i < 6; ++i) sol[i] = joint[i];
  } 
  for (int i = 0; i < 6; ++i) {
    printf("%f ", sol[i]);
    t.points[0].positions[i] = joint[i];
    t.points[1].positions[i] = sol[i]; 
    t.points[0].velocities[i] = 
    t.points[1].velocities[i] = 0;
  } printf("\n");
  t.points[0].time_from_start = ros::Duration(0);
  t.points[1].time_from_start = ros::Duration(calculate_time(joint, sol, factor));
  ROS_INFO("[%s] Execution time: %f seconds", ros::this_node::getName().c_str(), calculate_time(joint, sol, factor));
  return goal;
}
