#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // savePCDFileASCII
#include <pcl/point_types.h> // PointXYZ
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/common/transforms.h> // transformPointCloud
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/SetBool.h>
#include <visual_system/get_xyz.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

bool verbose; // If save point cloud as pcd
bool lock = false; // If lock data
int num_thres; // Points less than this threshold will be considered as empty workspace
double x_lower; // X lower bound, in hand coord.
double x_upper; // X upper bound, in hand coord.
double y_lower; // Y lower bound, in hand coord.
double y_upper; // Y upper bound, in hand coord.
double z_lower; // Z lower bound, in hand coord.
double z_upper; // Z upper bound, in hand coord.
const double Z_THRES_LOWER = 0.01f; // Z lower bound to check if workspace were empty
const double Z_THRES_UPPER = 0.20f; // Z upper bound to check if workspace were empty
std::vector<double> intrinsic; // [fx, fy, cx, cy]
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
Eigen::Matrix4f arm2cam_tf;
pcl::PointCloud<pcl::PointXYZRGB> pc;
pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;

Eigen::Matrix4f lookup_transform(void);
void callback_sub(const sensor_msgs::ImageConstPtr& color_image, 
                  const sensor_msgs::ImageConstPtr& depth_image,
                  const sensor_msgs::CameraInfoConstPtr& cam_info);
bool callback_get_pc(visual_system::get_pc::Request &req,
                     visual_system::get_pc::Response &res);
bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res);

int main(int argc, char** argv)
{
  intrinsic.resize(4);
  ros::init(argc, argv, "pixel_to_xyz");
  ros::NodeHandle nh, pnh("~");
  if(!pnh.getParam("x_lower", x_lower)) x_lower = -0.659f;
  if(!pnh.getParam("x_upper", x_upper)) x_upper = -0.273f;
  if(!pnh.getParam("y_lower", y_lower)) y_lower = -0.269f;
  if(!pnh.getParam("y_upper", y_upper)) y_upper =  0.114f;
  if(!pnh.getParam("z_lower", z_lower)) z_lower = -0.100f;
  if(!pnh.getParam("z_upper", z_upper)) z_upper =  0.200f;
  if(!pnh.getParam("verbose", verbose)) verbose = false;
  if(!pnh.getParam("num_thres", num_thres)) num_thres = 7000; // Points number greater than this value will be considered as not empty
  if(verbose)
    ROS_WARN("Save pointcloud for debuging");
  else
    ROS_WARN("Not save pointcloud");
  arm2cam_tf = lookup_transform();
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_lower, x_upper);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_lower, y_upper);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_lower, z_upper);
  // Message filter: color image, depth image, color camera info
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera/color/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, 
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo> \
                                                    MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback_sub, _1, _2, _3));
  ros::ServiceServer pc_service = pnh.advertiseService("get_pc", callback_get_pc);
  ros::ServiceServer check_empty_service = pnh.advertiseService("empty_state", callback_is_empty);
  ros::spin();
  return 0;
}

Eigen::Matrix4f lookup_transform(void){
  Eigen::Matrix4f eigen_mat = Eigen::Matrix4f::Identity();
  tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform("base_link", "camera1_color_optical_frame", ros::Time(0), ros::Duration(0.3));
    listener.lookupTransform("base_link", "camera1_color_optical_frame", ros::Time(0), stf);
  } catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Can't get transformation, shutdown...");
    ros::shutdown();
  }
  tf::Matrix3x3 tf_mat(stf.getRotation());
  eigen_mat(0, 0) = tf_mat[0].getX(); eigen_mat(1, 0) = tf_mat[1].getX(); eigen_mat(2, 0) = tf_mat[2].getX(); 
  eigen_mat(0, 1) = tf_mat[0].getY(); eigen_mat(1, 1) = tf_mat[1].getY(); eigen_mat(2, 1) = tf_mat[2].getY(); 
  eigen_mat(0, 2) = tf_mat[0].getZ(); eigen_mat(1, 2) = tf_mat[1].getZ(); eigen_mat(2, 2) = tf_mat[2].getZ(); 
  eigen_mat(0, 3) = stf.getOrigin().getX(); eigen_mat(1, 3) = stf.getOrigin().getY(); eigen_mat(2, 3) = stf.getOrigin().getZ(); 
  return eigen_mat;
}

void callback_sub(const sensor_msgs::ImageConstPtr& color_image, 
                  const sensor_msgs::ImageConstPtr& depth_image, 
                  const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if(lock) return;
  intrinsic[0] = cam_info->K[0]; // fx
  intrinsic[1] = cam_info->K[4]; // fy
  intrinsic[2] = cam_info->K[2]; // cx
  intrinsic[3] = cam_info->K[5]; // cy
  try{
    color_img_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }  
  try{
    depth_img_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
  pc.clear();
  for(int x=0; x<color_img_ptr->image.cols; ++x){ // 0~639
    for(int y=0; y<color_img_ptr->image.rows; ++y){ // 0~479
      auto depth = depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
      pcl::PointXYZRGB p;
      p.z = depth*0.001; // mm to m
      p.x = (x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
      p.y = (y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
      p.r = color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
      p.g = color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
      p.b = color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
      pc.points.push_back(p);
    }
  }
}

bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res)
{
  pcl::PointCloud<pcl::PointXYZRGB> pc, pc_filtered;
  pcl::fromROSMsg(req.input_pc, pc);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(pc.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(Z_THRES_LOWER, Z_THRES_UPPER);
  pass.filter(pc_filtered);
  if(pc_filtered.points.size()<num_thres) res.is_empty.data = true;
  else res.is_empty.data = false;
  return true;
}

bool callback_get_pc(visual_system::get_pc::Request  &req, 
                     visual_system::get_pc::Response &res)
{
  lock = true;
  ros::Time ts = ros::Time::now();
  // Transform points to hand coordinate
  pcl::transformPointCloud(pc, pc, arm2cam_tf);
  // Pass through XYZ
  pass_x.setInputCloud(pc.makeShared());
  pass_x.filter(pc);
  pass_y.setInputCloud(pc.makeShared());
  pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);
  /*
  for(auto &p: pc){
    double x = p.x,
           y = p.y;
    p.x = y; p.y = -x; 
  }*/
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pc, pc2);
  pc2.header.frame_id = "base_link";
  res.pc = pc2;
  if(verbose and !req.file_name.empty()){
    std::string pc_name;
    // Make sure given filename has extension
    std::size_t pos = req.file_name.find(".pcd");
    if(pos == std::string::npos)
      pc_name = req.file_name + ".pcd";
    else pc_name = req.file_name;
    // Make sure the directory exists
    for(size_t i=0; i<pc_name.length(); ++i)
      if(pc_name[i]=='/') pos = i;
    boost::filesystem::path p(pc_name.substr(0, pos));
    if(!boost::filesystem::exists(p))
      create_directories(p);
    pcl::io::savePCDFileASCII(pc_name, pc);
  }
  lock = false;
  return true;
}
