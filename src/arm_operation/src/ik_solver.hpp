#include <iomanip>
#include <cmath>
#include <cstring>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ur_kin.h>

// Output angle in range [-pi, pi]
inline double validAngle(double angle){
  if(abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
  if(angle > M_PI) angle -= 2*M_PI;
  else if(angle < -M_PI) angle += 2*M_PI;
  return angle;
}

// Convert pose to transformation matrix
void PoseToDH(geometry_msgs::Pose pose, double *T){
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

// Perform IK, return solution string
std::string PerformIK(double tool_length, geometry_msgs::Pose target_pose){
  double T[16] = {0};
  std::stringstream ss;
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6];
  int sols = ur_kinematics::inverse(T, q_sols);
  if(sols==0 or std::isnan(q_sols[0])) return "No solution found!";
  ss << "There are: " << sols << " solutions found.\n\n"; 
  for (int i = 0; i < sols; ++i) {
    // Preprocess joint angle to -pi ~ pi
    for (int j = 0; j < 6; ++j) {
      q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
    } 
    for (int j = 0; j < 6; ++j){
      ss << "Joint" << j+1 << ": " << std::fixed << std::setprecision(6) << q_sols[6*i+j] << " ";
    }
    ss << "\n";
  }
  return ss.str();
}

// UI bridge
std::string UIBridge(double x, double y, double z, 
                     double qx, double qy, double qz, double qw,
                     double tool_length)
{
  geometry_msgs::Pose pose;
  pose.position.x = x; pose.position.y = y; pose.position.z = z; 
  pose.orientation.x = qx; pose.orientation.y = qy; 
  pose.orientation.z = qz; pose.orientation.w = qw;
  return PerformIK(tool_length, pose);
}
