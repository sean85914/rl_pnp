#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_cams_tf");
  std::vector<double> cam1_tf, cam2_tf;
  cam1_tf.resize(7); // x y z qx qy qz qw
  cam2_tf.resize(7); // x y z qx qy qz qw
  ros::NodeHandle nh, pnh("~");
  if(!pnh.getParam("cam1_tf", cam1_tf)){
    ROS_ERROR("Can't get camera1 information, exit...");
    ros::shutdown();
  }
  if(!pnh.getParam("cam2_tf", cam2_tf)){
    ROS_ERROR("Can't get camera2 information, exit...");
    ros::shutdown();
  }
  static tf::TransformBroadcaster br;
  tf::Quaternion cam1_quat(cam1_tf[3], cam1_tf[4], cam1_tf[5], cam1_tf[6]),
                 cam2_quat(cam2_tf[3], cam2_tf[4], cam2_tf[5], cam2_tf[6]);
  tf::Vector3 cam1_origin(cam1_tf[0], cam1_tf[1], cam1_tf[2]),
             cam2_origin(cam2_tf[0], cam2_tf[1], cam2_tf[2]);
  tf::Transform tf_1(cam1_quat, cam1_origin),
                tf_2(cam2_quat, cam2_origin);
  ros::Rate r(10); // 10 Hz = 100 ms
  while(ros::ok()){
    br.sendTransform(tf::StampedTransform(tf_1, ros::Time::now(), "base_link", "camera1_link"));
    br.sendTransform(tf::StampedTransform(tf_2, ros::Time::now(), "camera1_link", "camera2_link"));
    r.sleep();
  }
}
