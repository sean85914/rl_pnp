#include <thread> // std::thread
#include <mutex> // std::mutex
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
#include <pcl/filters/conditional_removal.h> // pcl::ConditionalRemoval
#include <pcl/kdtree/kdtree_flann.h> // pcl::KdTreeFLANN
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
#include <pcl_ros/impl/transforms.hpp> // pcl_ros::transformPointCloud
// SRV
#include <std_srvs/SetBool.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
#include <visual_system/check_grasp_success.h>
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
const int thread_num = 16; // Number of threads
double factor; // Voxel grid factor
double empty_thres; // Threshold rate for determining if the bin is empty
double success_thres; // Threshold rate for determining if the grasp is successful
double x_lower; // X lower bound, in hand coord.
double x_upper; // X upper bound, in hand coord.
double y_lower; // Y lower bound, in hand coord.
double y_upper; // Y upper bound, in hand coord.
double z_lower; // Z lower bound, in hand coord.
double z_upper; // Z upper bound, in hand coord.
std::string cam1_name, cam2_name; // Camera prefix
std::string node_ns; // Node namespace
std::mutex mtx; // Mutex lock for threading
tf::Transform arm2cam1, arm2cam2;
pcl::PointCloud<pcl::PointXYZRGB> pc_cam1, pc_cam2;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>> global_vec; // Pointcloud placeholder for threading
pcl::VoxelGrid<pcl::PointXYZRGB> vg;
pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond;
pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
void lookup_transform(std::string target_frame);
ros::Publisher pub_pc; // Publish plane pointcloud for debuging
void callback(const sensor_msgs::PointCloud2::ConstPtr);
bool callback_get_pc(visual_system::get_pc::Request &req,
                     visual_system::get_pc::Response &res);
bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res);
void check_param_cb(const ros::TimerEvent&);
void workers(int id, pcl::PointCloud<pcl::PointXYZRGB> sub_pc); // Thread target
bool callback_grasp_success(visual_system::check_grasp_success::Request  &req,
                            visual_system::check_grasp_success::Response &res);

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
  if(!pnh.getParam("empty_thres", empty_thres)) empty_thres = 0.95f;
  if(!pnh.getParam("success_thres", success_thres)) success_thres = 0.10f;
  if(!pnh.getParam("verbose", verbose)) verbose = false;
  if(!pnh.getParam("down_sample", down_sample)) down_sample = false;
  if(!pnh.getParam("use_two_cam", use_two_cam)) use_two_cam = true;
  if(!pnh.getParam("cam1_name", cam1_name)) cam1_name = "camera1";
  if(!pnh.getParam("cam2_name", cam2_name)) cam2_name = "camera2";
  lookup_transform(cam1_name+"_color_optical_frame");
  if(verbose)
    ROS_WARN("Save pointcloud for debuging");
  else
    ROS_WARN("Not save pointcloud");
  global_vec.resize(thread_num*(use_two_cam?2:1));
  if(use_two_cam) lookup_transform(cam2_name+"_color_optical_frame");
  if(down_sample) 
    vg.setLeafSize((x_upper-x_lower)/resolution/factor, (x_upper-x_lower)/resolution/factor, (x_upper-x_lower)/resolution/factor);
  // Setup conditions
  range_cond = pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, x_lower)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, x_upper)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, y_lower)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, y_upper)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, z_lower)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, z_upper)));
  condrem.setCondition(range_cond);
  pub_pc = pnh.advertise<sensor_msgs::PointCloud2>("plane_pc", 1);
  // Setup subscribers
  ros::Subscriber sub_cam1_pc = pnh.subscribe("cam1_pc", 1, callback),
                  sub_cam2_pc = pnh.subscribe("cam2_pc", 1, callback);
  // Setup service server
  ros::ServiceServer pc_service = pnh.advertiseService("get_pc", callback_get_pc);
  ros::ServiceServer check_empty_service = pnh.advertiseService("empty_state", callback_is_empty);
  ros::ServiceServer check_grasp_success_service = pnh.advertiseService("grasp_state", callback_grasp_success);
  ros::Timer check_param_timer = pnh.createTimer(ros::Duration(1.0), check_param_cb);
  ros::spin();
  // Save YAML after terminating
  std::string package_path = ros::package::getPath("visual_system"),
              yaml_path = package_path + "/config/param_config.yaml",
              cmd_str = "rosparam dump " + yaml_path + " " + node_ns;
  system(cmd_str.c_str());
  return 0;
}

void lookup_transform(std::string target_frame){
  Eigen::Matrix4f eigen_mat = Eigen::Matrix4f::Identity();
  tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform("base_link", target_frame, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_link", target_frame, ros::Time(0), stf);
    if(target_frame=="camera1_color_optical_frame"){
      arm2cam1 = tf::Transform(stf.getRotation(), stf.getOrigin());
    } else{
      arm2cam2 = tf::Transform(stf.getRotation(), stf.getOrigin());
    }
  } catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Can't get transformation, shutdown...");
    ros::shutdown();
  }
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
  seg.setDistanceThreshold(0.005f);
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  pcl::fromROSMsg(req.input_pc, pc);
  seg.setInputCloud(pc.makeShared());
  seg.segment(*inliers, *coefficients);
  double ratio = double(inliers->indices.size())/pc.points.size();
  ROS_INFO("Ratio: %f", ratio);
  if(ratio>=empty_thres){
    res.is_empty.data = true;
  } else res.is_empty.data = false;
  sensor_msgs::PointCloud2 pc_out;
  pcl::PointCloud<pcl::PointXYZRGB> plane_pc(pc, inliers->indices);
  pcl::toROSMsg(plane_pc, pc_out);
  pc_out.header.frame_id = "base_link";
  pub_pc.publish(pc_out);
  return true;
}

bool callback_get_pc(visual_system::get_pc::Request  &req, 
                     visual_system::get_pc::Response &res)
{
  ros::Time ts = ros::Time::now();
  // Downsample if necessary
  pcl::PointCloud<pcl::PointXYZRGB> pc_cam1_vg, pc_cam2_vg;
  if(down_sample){
    vg.setInputCloud(pc_cam1.makeShared());
    vg.filter(pc_cam1_vg);
    if(use_two_cam){
      vg.setInputCloud(pc_cam2.makeShared());
      vg.filter(pc_cam2_vg);
    }
  }else{
    pc_cam1_vg = pc_cam1;
    pc_cam2_vg = pc_cam2;
  }
  std::thread thread_arr[thread_num*(use_two_cam?2:1)];
  for(int i=0; i<thread_num; ++i){
    std::vector<int> indices;
    int size = pc_cam1_vg.points.size();
    for(int j=size/thread_num*i; j<size/thread_num*(i+1); ++j){
      indices.push_back(j);
    }
    pcl::PointCloud<pcl::PointXYZRGB> sub_pc(pc_cam1_vg, indices);
    thread_arr[i] = std::thread(workers, i, sub_pc);
    if(use_two_cam){
      pcl::PointCloud<pcl::PointXYZRGB> sub_pc_2(pc_cam2_vg, indices);
      thread_arr[thread_num+i] = std::thread(workers, thread_num+i, sub_pc);
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB> pc_in_range;
  for(int i=0; i<thread_num*(use_two_cam?2:1); ++i){
    thread_arr[i].join();
    if(global_vec[i].size()>0)
      pc_in_range += global_vec[i];
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
  if(ros::param::get(node_ns+"/success_thres", tmp)){
    if(tmp!=success_thres){
      ROS_INFO("success_thres set from %f to %f", success_thres, tmp);
      success_thres = tmp;
    }
  }
}

void workers(int id, pcl::PointCloud<pcl::PointXYZRGB> sub_pc){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Transform pointcloud
  pcl_ros::transformPointCloud(sub_pc, *transform, arm2cam1);
  mtx.lock();
  // Using conditional removal to filter points and placing into placeholder
  condrem.setInputCloud(transform);
  condrem.filter(global_vec[id]);
  mtx.unlock();
}

bool callback_grasp_success(visual_system::check_grasp_success::Request  &req,
                            visual_system::check_grasp_success::Response &res)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prior_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                         post_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(req.prior_pcd_str, *prior_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(req.post_pcd_str,  *post_cloud);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> prior_tree(false), post_tree(false);
  prior_tree.setInputCloud(prior_cloud);
  post_tree.setInputCloud(post_cloud);
  pcl::PointXYZRGB searchPoint;
  searchPoint.x = req.operated_position.x;
  searchPoint.y = req.operated_position.y;
  searchPoint.z = req.operated_position.z;
  std::vector<int> prior_inliers, post_inliers;
  std::vector<float> _;
  double radius = 0.02; // 2cm
  prior_tree.radiusSearch(searchPoint, radius, prior_inliers, _);
  post_tree.radiusSearch(searchPoint, radius, post_inliers, _);
  double rate = post_inliers.size()/(double)prior_inliers.size();
  if(rate<=success_thres){
    res.is_success = true;
  } else res.is_success = false;
  ROS_INFO("Prior: %d \t Posterior: %d \t Success: %s", (int)prior_inliers.size(), (int)post_inliers.size(), (res.is_success?"True":"False"));
  return true;
}
