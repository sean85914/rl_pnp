#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> // pcl::VoxelGrid
#include <pcl/ModelCoefficients.h> // pcl::ModelCoefficients
#include <pcl/io/pcd_io.h> // pcl::io::savePCDFileASCII, pcl::io::savePCDFileBinary
#include <pcl/point_types.h> // pcl::PointXYZ
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
// SRV
#include <std_srvs/SetBool.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

bool verbose; // If save point cloud as pcd
bool use_two_cam; // If enable camera2
bool down_sample; // If downsampling
int resolution; // Heightmap resolution
double factor; // Voxel grid factor
double empty_thres; // Threshold rate for determining if the bin is empty
double x_lower; // X lower bound, in hand coord.
double x_upper; // X upper bound, in hand coord.
double y_lower; // Y lower bound, in hand coord.
double y_upper; // Y upper bound, in hand coord.
double z_lower; // Z lower bound, in hand coord.
double z_upper; // Z upper bound, in hand coord.
std::string cam1_name, cam2_name; // Camera prefix
std::string node_ns; // Node namespace
Eigen::Matrix4f arm2cam1_tf, arm2cam2_tf;
pcl::PointCloud<pcl::PointXYZRGB> pc_cam1, pc_cam2;
pcl::VoxelGrid<pcl::PointXYZRGB> vg;
pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
Eigen::Matrix4f lookup_transform(std::string target_frame);
void callback(const sensor_msgs::PointCloud2::ConstPtr);
bool callback_get_pc(visual_system::get_pc::Request &req,
                     visual_system::get_pc::Response &res);
bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res);
void check_param_cb(const ros::TimerEvent&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "combine_pc_node");
  ros::NodeHandle nh, pnh("~");
  node_ns = pnh.getNamespace();
  // Setup parameters
  if(!pnh.getParam("x_lower", x_lower)) x_lower =  0.700f;
  if(!pnh.getParam("x_upper", x_upper)) x_upper =  1.000f;
  if(!pnh.getParam("y_lower", y_lower)) y_lower = -0.500f;
  if(!pnh.getParam("y_upper", y_upper)) y_upper = -0.200f;
  if(!pnh.getParam("z_lower", z_lower)) z_lower = -0.100f;
  if(!pnh.getParam("z_upper", z_upper)) z_upper =  0.200f;
  if(!pnh.getParam("resolution", resolution)) resolution = 224;
  if(!pnh.getParam("factor", factor)) factor = 1.0f;
  if(!pnh.getParam("empty_thres", empty_thres)) empty_thres = 1.0f;
  if(!pnh.getParam("verbose", verbose)) verbose = false;
  if(!pnh.getParam("down_sample", down_sample)) down_sample = false;
  if(!pnh.getParam("use_two_cam", use_two_cam)) use_two_cam = true;
  if(!pnh.getParam("cam1_name", cam1_name)) cam1_name = "camera1";
  if(!pnh.getParam("cam2_name", cam2_name)) cam2_name = "camera2";
  arm2cam1_tf = lookup_transform(cam1_name+"_color_optical_frame");
  if(verbose)
    ROS_WARN("Save pointcloud for debuging");
  else
    ROS_WARN("Not save pointcloud");
  if(use_two_cam) arm2cam2_tf = lookup_transform(cam2_name+"_color_optical_frame");
  if(down_sample) 
    vg.setLeafSize((x_upper-x_lower)/resolution/factor, (x_upper-x_lower)/resolution/factor, (x_upper-x_lower)/resolution/factor);
  // Setup pass through filter
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_lower, x_upper);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_lower, y_upper);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_lower, z_upper);
  // Setup subscribers
  ros::Subscriber sub_cam1_pc = pnh.subscribe("cam1_pc", 1, callback),
                  sub_cam2_pc = pnh.subscribe("cam2_pc", 1, callback);
  // Setup service server
  ros::ServiceServer pc_service = pnh.advertiseService("get_pc", callback_get_pc);
  ros::ServiceServer check_empty_service = pnh.advertiseService("empty_state", callback_is_empty);
  ros::Timer check_param_timer = pnh.createTimer(ros::Duration(1.0), check_param_cb);
  ros::spin();
  // Save YAML after terminating
  std::string package_path = ros::package::getPath("visual_system"),
              yaml_path = package_path + "/config/param_config.yaml",
              cmd_str = "rosparam dump " + yaml_path + " " + node_ns;
  system(cmd_str.c_str());
  return 0;
}

Eigen::Matrix4f lookup_transform(std::string target_frame){
  Eigen::Matrix4f eigen_mat = Eigen::Matrix4f::Identity();
  tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform("base_link", target_frame, ros::Time(0), ros::Duration(1.0));
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

void callback(const sensor_msgs::PointCloud2::ConstPtr msg){
  if(msg->header.frame_id == cam1_name+"_color_optical_frame"){
    pcl::fromROSMsg(*msg, pc_cam1);
  } else if(msg->header.frame_id == cam2_name+"_color_optical_frame"){
    if(use_two_cam)
      pcl::fromROSMsg(*msg, pc_cam2);
  }
}

bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res)
{
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
  double ratio = double(inliers->indices.size())/pc.points.size();
  ROS_INFO("Ratio: %f", ratio);
  if(ratio>=empty_thres){
    res.is_empty.data = true;
  } else res.is_empty.data = false;
  return true;
}

bool callback_get_pc(visual_system::get_pc::Request  &req, 
                     visual_system::get_pc::Response &res)
{
  ros::Time ts = ros::Time::now();
  // Transform points to hand coordinate
  pcl::PointCloud<pcl::PointXYZRGB> pc_in_range, pc_cam1_transformed, pc_cam2_transformed;
  pcl::transformPointCloud(pc_cam1, pc_cam1_transformed, arm2cam1_tf);
  pcl::transformPointCloud(pc_cam2, pc_cam2_transformed, arm2cam2_tf);
  pc_in_range = pc_cam1_transformed + pc_cam2_transformed;
  // Pass through XYZ
  pass_x.setInputCloud(pc_in_range.makeShared());
  pass_x.filter(pc_in_range);
  pass_y.setInputCloud(pc_in_range.makeShared());
  pass_y.filter(pc_in_range);
  pass_z.setInputCloud(pc_in_range.makeShared());
  pass_z.filter(pc_in_range);
  if(down_sample){
    vg.setInputCloud(pc_in_range.makeShared());
    vg.filter(pc_in_range);
  }
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
    pcl::io::savePCDFileBinary(pc_name, pc_in_range);
  }
  ROS_INFO("Done, %d points in range, spent %f seconds to process", (int)pc_in_range.size(), (ros::Time::now()-ts).toSec());
  return true;
}

void check_param_cb(const ros::TimerEvent& event){
  double tmp;
  if(ros::param::get(node_ns+"/factor", tmp)){
    if(tmp!=factor){
      ROS_INFO("factor set from %f to %f", factor, tmp);
      factor = tmp;
    }
  }
  if(ros::param::get(node_ns+"/empty_thres", tmp)){
    if(tmp!=empty_thres){
      ROS_INFO("empty_thres set from %f to %f", empty_thres, tmp);
      empty_thres = tmp;
    }
  }
}
