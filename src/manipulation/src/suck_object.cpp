#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
// SRV
#include <arm_operation/target_pose.h>
#include <visual_system/target_pose.h>
// MSG
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

class SuckObj{
 private:
  bool motion;
  ros::NodeHandle nh_, pnh_;
  // Service client
  ros::ServiceClient visual_system_client;
  ros::ServiceClient arm_operation_client;
  arm_operation::target_pose pose_req;
  visual_system::target_pose get_pose_req;
  void start(void);
 public:
  SuckObj(ros::NodeHandle, ros::NodeHandle);
};

SuckObj::SuckObj(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
  ros::service::waitForService("/ur3_control_server/ur_control/goto_pose");
  arm_operation_client = pnh_.serviceClient<arm_operation::target_pose>("/ur3_control_server/ur_control/goto_pose");
  ros::service::waitForService("/get_target_pc_node/get_target_info");
  visual_system_client = pnh_.serviceClient<visual_system::target_pose>("/get_target_pc_node/get_target_info");
  ROS_INFO("Service conneted!");
  if(!pnh_.getParam("motion", motion)) motion = false; ROS_INFO("motion: %s", (motion==true?"true":"false"));
  pose_req.request.factor = 0.5f;
  start();
  ros::shutdown();
}

void SuckObj::start(void){
  visual_system_client.call(get_pose_req);
  ROS_INFO("Get target centroid: (%f, %f, %f) with normal (%f, %f, %f)", 
           get_pose_req.response.centroid.x,
           get_pose_req.response.centroid.y,
           get_pose_req.response.centroid.z,
           get_pose_req.response.normal.x,
           get_pose_req.response.normal.y,
           get_pose_req.response.normal.z);
  geometry_msgs::Point centroid = get_pose_req.response.centroid;
  geometry_msgs::Point normal = get_pose_req.response.normal;
  pose_req.request.target_pose.position.x = centroid.x; // + normal.x;
  pose_req.request.target_pose.position.y = centroid.y; // + normal.y;
  pose_req.request.target_pose.position.z = centroid.z; // + normal.z;
  tf::Vector3 x_vec(-normal.x, -normal.y, -normal.z),
              z_vec(-1.0f, 0.0f, 0.0f),
              y_vec = z_vec.cross(x_vec);
  tf::Matrix3x3 rot_mat(x_vec.getX(), y_vec.getX(), z_vec.getX(),
                        x_vec.getY(), y_vec.getY(), z_vec.getY(),
                        x_vec.getZ(), y_vec.getZ(), z_vec.getZ());
  tf::Quaternion quat; rot_mat.getRotation(quat);
  pose_req.request.target_pose.orientation.x = quat.getX();
  pose_req.request.target_pose.orientation.y = quat.getY();
  pose_req.request.target_pose.orientation.z = quat.getZ();
  pose_req.request.target_pose.orientation.w = quat.getW();
  ROS_INFO("Target pose: \n\tPosition: [%f, %f, %f]\n\tQuaternion: [%f, %f, %f, %f]",
           pose_req.request.target_pose.position.x,
           pose_req.request.target_pose.position.y,
           pose_req.request.target_pose.position.z,
           pose_req.request.target_pose.orientation.x,
           pose_req.request.target_pose.orientation.y,
           pose_req.request.target_pose.orientation.z,
           pose_req.request.target_pose.orientation.w);
  if(motion) arm_operation_client.call(pose_req);
  //  Broadcast transform to debug
  /*
  tf::Vector3 target_pose(pose_req.request.target_pose.position.x,
                          pose_req.request.target_pose.position.y,
                          pose_req.request.target_pose.position.z);
  tf::Transform transform(quat, target_pose);
  static tf::TransformBroadcaster br;
  ros::Time now = ros::Time::now();
  while((ros::Time::now()-now).toSec() < 3.0)
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "target"));
  */
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh, pnh("~");
  SuckObj foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
