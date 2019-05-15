#include "ur_control_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_control_server_node");
  ros::NodeHandle nh, pnh("~");
  RobotArm ur(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
