#include <iostream>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ur_kin.h>

double tool_length;

double validAngle(double angle){
  if(abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
  if(angle > M_PI) angle -= 2*M_PI;
  else if(angle < -M_PI) angle += 2*M_PI;
  return angle;
}

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

int PerformIK(geometry_msgs::Pose target_pose){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6];
  int sols = ur_kinematics::inverse(T, q_sols);
  std::cout << "There are " << sols << " solutions.\n";
  for (int i = 0; i < sols; ++i) {
    // Preprocess joint angle to -pi ~ pi
    for (int j = 0; j < 6; ++j) {
      q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
    } 
    for (int j = 0; j < 6; ++j){
      std::cout << q_sols[6*i+j] << " ";
    }
    std::cout << "\n";
  }
  return sols;
}

void printHelp(void){
  std::cout << "\033[1;31mNot enough arguments, please provides: tool length x y z qx qy qz qw]\033[0m\n";
}

int main(int argc, char** argv)
{
  if(argc!=9){
    printHelp(); return -1;
  }
  tool_length = atof(argv[1]);
  geometry_msgs::Pose p;
  p.position.x = atof(argv[2]);
  p.position.y = atof(argv[3]);
  p.position.z = atof(argv[4]);
  p.orientation.x = atof(argv[5]);
  p.orientation.y = atof(argv[6]);
  p.orientation.z = atof(argv[7]);
  p.orientation.w = atof(argv[8]);
  PerformIK(p);
  return 0;
}

