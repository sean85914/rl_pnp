#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h> // pcl::ModelCoefficients
#include <pcl/io/pcd_io.h> // pcl::io::savePCDFileASCII, pcl::io::savePCDFileBinary
#include <pcl/point_types.h> // pcl::PointXYZ
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
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
int num_thres; // Points less than this threshold will be considered as empty workspace
double x_lower; // X lower bound, in hand coord.
double x_upper; // X upper bound, in hand coord.
double y_lower; // Y lower bound, in hand coord.
double y_upper; // Y upper bound, in hand coord.
double z_lower; // Z lower bound, in hand coord.
double z_upper; // Z upper bound, in hand coord.
double z_thres_lower; // Z lower bound to check if workspace were empty
double z_thres_upper; // Z upper bound to check if workspace were empty
std::string cam_name, on_hand_cam_name;
std::vector<double> cam_intrinsic, on_hand_cam_intrinsic; // [fx, fy, cx, cy]
cv_bridge::CvImagePtr cam_color_img_ptr, cam_depth_img_ptr, on_hand_cam_color_img_ptr, on_hand_cam_depth_img_ptr;
Eigen::Matrix4f arm2cam_tf, arm2on_hand_cam;
pcl::PointCloud<pcl::PointXYZRGB> pc_cam, pc_on_hand_cam;
pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;

Eigen::Matrix4f lookup_transform(std::string target_frame);
void callback_sub(const sensor_msgs::ImageConstPtr& color_image, 
                  const sensor_msgs::ImageConstPtr& depth_image,
                  const sensor_msgs::CameraInfoConstPtr& cam_info);
bool callback_get_pc(visual_system::get_pc::Request &req,
                     visual_system::get_pc::Response &res);
bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res);

int main(int argc, char** argv)
{
  cam_intrinsic.resize(4); on_hand_cam_intrinsic.resize(4);
  ros::init(argc, argv, "pixel_to_xyz");
  ros::NodeHandle nh, pnh("~");
  if(!pnh.getParam("x_lower", x_lower)) x_lower = -0.659f;
  if(!pnh.getParam("x_upper", x_upper)) x_upper = -0.273f;
  if(!pnh.getParam("y_lower", y_lower)) y_lower = -0.269f;
  if(!pnh.getParam("y_upper", y_upper)) y_upper =  0.114f;
  if(!pnh.getParam("z_lower", z_lower)) z_lower = -0.100f;
  if(!pnh.getParam("z_upper", z_upper)) z_upper =  0.200f;
  if(!pnh.getParam("z_thres_lower", z_thres_lower)) z_thres_lower = 0.02f;
  if(!pnh.getParam("z_thres_upper", z_thres_upper)) z_thres_upper = 0.20f;
  if(!pnh.getParam("verbose", verbose)) verbose = false;
  if(!pnh.getParam("num_thres", num_thres)) num_thres = 7000; // Points number greater than this value will be considered as not empty
  if(verbose)
    ROS_WARN("Save pointcloud for debuging");
  else
    ROS_WARN("Not save pointcloud");
  if(!pnh.getParam("cam_name", cam_name)) cam_name = "camera1";
  if(!pnh.getParam("on_hand_cam_name", on_hand_cam_name)) on_hand_cam_name = "on_hand_cam";
  arm2cam_tf = lookup_transform(cam_name+"_color_optical_frame");
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_lower, x_upper);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_lower, y_upper);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_lower, z_upper);
  // Message filter: color image, depth image, color camera info
  message_filters::Subscriber<sensor_msgs::Image> cam_color_image_sub(nh, cam_name+"/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> cam_depth_image_sub(nh, cam_name+"/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, cam_name+"/color/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::Image> on_hane_cam_color_image_sub(nh, on_hand_cam_name+"/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> on_hand_cam_depth_image_sub(nh, on_hand_cam_name+"/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> on_hand_cam_info_sub(nh, on_hand_cam_name+"/color/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, 
                                                    sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo> \
                                                    MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_cam(MySyncPolicy(10), cam_color_image_sub, cam_depth_image_sub, cam_info_sub);
  message_filters::Synchronizer<MySyncPolicy> sync_on_hand_cam(MySyncPolicy(10), on_hane_cam_color_image_sub, on_hand_cam_depth_image_sub, on_hand_cam_info_sub);
  sync_cam.registerCallback(boost::bind(&callback_sub, _1, _2, _3));
  sync_on_hand_cam.registerCallback(boost::bind(&callback_sub, _1, _2, _3));
  ros::ServiceServer pc_service = pnh.advertiseService("get_pc", callback_get_pc);
  ros::ServiceServer check_empty_service = pnh.advertiseService("empty_state", callback_is_empty);
  ros::spin();
  return 0;
}

Eigen::Matrix4f lookup_transform(std::string target_frame){
  Eigen::Matrix4f eigen_mat = Eigen::Matrix4f::Identity();
  tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform("base_link", target_frame, ros::Time(0), ros::Duration(0.3));
    listener.lookupTransform("base_link", target_frame, ros::Time(0), stf);
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
  if(cam_info->header.frame_id == cam_name+"_color_optical_frame"){
    cam_intrinsic[0] = cam_info->K[0]; // fx
    cam_intrinsic[1] = cam_info->K[4]; // fy
    cam_intrinsic[2] = cam_info->K[2]; // cx
    cam_intrinsic[3] = cam_info->K[5]; // cy
    try{
      cam_color_img_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what()); return;
    }  
    try{
      cam_depth_img_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch(cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what()); return;
    }
    pc_cam.clear();
    for(int x=0; x<cam_color_img_ptr->image.cols; ++x){ // 0~639
      for(int y=0; y<cam_color_img_ptr->image.rows; ++y){ // 0~479
        auto depth = cam_depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
        pcl::PointXYZRGB p;
        p.z = depth*0.001; // mm to m
        p.x = (x-cam_intrinsic[2])/cam_intrinsic[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-cam_intrinsic[3])/cam_intrinsic[1]*p.z; // y = (v-cy)/fy*z
        p.r = cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
        p.g = cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
        p.b = cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
        pc_cam.points.push_back(p);
      }
    }
  }else if(cam_info->header.frame_id == on_hand_cam_name+"_color_optical_frame"){
    on_hand_cam_intrinsic[0] = cam_info->K[0]; // fx
    on_hand_cam_intrinsic[1] = cam_info->K[4]; // fy
    on_hand_cam_intrinsic[2] = cam_info->K[2]; // cx
    on_hand_cam_intrinsic[3] = cam_info->K[5]; // cy
    try{
      on_hand_cam_color_img_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what()); return;
    }  
    try{
      on_hand_cam_depth_img_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch(cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what()); return;
    }
    pc_on_hand_cam.clear();
    for(int x=0; x<on_hand_cam_color_img_ptr->image.cols; ++x){ // 0~639
      for(int y=0; y<on_hand_cam_color_img_ptr->image.rows; ++y){ // 0~479
        auto depth = on_hand_cam_depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
        pcl::PointXYZRGB p;
        p.z = depth*0.001; // mm to m
        p.x = (x-on_hand_cam_intrinsic[2])/on_hand_cam_intrinsic[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-on_hand_cam_intrinsic[3])/on_hand_cam_intrinsic[1]*p.z; // y = (v-cy)/fy*z
        p.r = on_hand_cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
        p.g = on_hand_cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
        p.b = on_hand_cam_color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
        pc_on_hand_cam.points.push_back(p);
      }
    }
  }
}

bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res)
{
  /*pcl::PointCloud<pcl::PointXYZRGB> pc, pc_filtered;
  pcl::fromROSMsg(req.input_pc, pc);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(pc.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_thres_lower, z_thres_upper);
  pass.filter(pc_filtered);
  ROS_INFO("[%s] Points in range: %d", ros::this_node::getName().c_str(), (int)pc_filtered.size());
  if(pc_filtered.points.size()<=num_thres) {res.is_empty.data = true; ROS_INFO("\033[0;33mWorkspace is empty\033[0m");}
  else {res.is_empty.data = false; ROS_INFO("\033[0;33mWorkspace is not empty\033[0m");}
  return true;*/
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  pcl::fromROSMsg(req.input_pc, pc);
  seg.setInputCloud(pc.makeShared());
  seg.segment(*inliers, *coefficients);
  ROS_INFO("Ratio: %f", double(inliers->indices.size())/pc.points.size());
  res.is_empty.data = true;
  return true;
}

bool callback_get_pc(visual_system::get_pc::Request  &req, 
                     visual_system::get_pc::Response &res)
{
  ros::Time ts = ros::Time::now();
  // Get transformation from base_link to on_hand_cam
  arm2on_hand_cam = lookup_transform(on_hand_cam_name+"_color_optical_frame");
  // Transform points to hand coordinate
  pcl::PointCloud<pcl::PointXYZRGB> pc_in_range, pc_cam_transformed, pc_on_hand_cam_transformed;
  pcl::transformPointCloud(pc_cam, pc_cam_transformed, arm2cam_tf);
  pcl::transformPointCloud(pc_on_hand_cam, pc_on_hand_cam_transformed, arm2on_hand_cam);
  pc_in_range = pc_cam_transformed + pc_on_hand_cam_transformed;
  // Pass through XYZ
  pass_x.setInputCloud(pc_in_range.makeShared());
  pass_x.filter(pc_in_range);
  pass_y.setInputCloud(pc_in_range.makeShared());
  pass_y.filter(pc_in_range);
  pass_z.setInputCloud(pc_in_range.makeShared());
  pass_z.filter(pc_in_range);
  ROS_INFO("%d points in range", (int)pc_in_range.size());
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pc_in_range, pc2);
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
    pcl::io::savePCDFileASCII(pc_name, pc_in_range);
  }
  return true;
}
