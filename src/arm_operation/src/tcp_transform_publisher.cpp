#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tcp_transform_publisher");
  ros::NodeHandle pnh("~");
  bool has_gripper, has_suction;
  std::vector<double> suction;
  std::vector<double> gripper;
  if(!pnh.getParam("suction", suction)) has_suction = false; else has_suction = true;
  if(!pnh.getParam("gripper", gripper)) has_gripper = false; else has_gripper = true;
  if(suction.size()!=3) has_suction = false;
  if(gripper.size()!=3) has_gripper = false;
  ROS_INFO("*******************************************");
  if(has_suction) ROS_INFO("Suction TCP: [%f, %f, %f]", suction[0], suction[1], suction[2]);
  if(has_gripper) ROS_INFO("Gripper TCP: [%f, %f, %f]", gripper[0], gripper[1], gripper[2]);
  if(!has_suction and !has_gripper){
    ROS_WARN("No parameters got, shutdown the node"); ros::shutdown(); 
  }
  ROS_INFO("*******************************************");
  tf::Transform transform_suction, transform_gripper;
  if(has_suction){ 
    transform_suction.setOrigin(tf::Vector3(suction[0], suction[1], suction[2]));
    transform_suction.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  }
  if(has_gripper){
    transform_gripper.setOrigin(tf::Vector3(gripper[0], gripper[1], gripper[2]));
    transform_gripper.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  }
  ros::Rate r(100); // 100hz
  while(ros::ok()){
    static tf::TransformBroadcaster br;
    if(has_suction)
      br.sendTransform(tf::StampedTransform(transform_suction, ros::Time::now(),
                                            "ee_link", "tcp_suction"));
    if(has_gripper)
      br.sendTransform(tf::StampedTransform(transform_gripper, ros::Time::now(),
                                            "ee_link", "tcp_gripper"));
    r.sleep();
  } return 0;
}
