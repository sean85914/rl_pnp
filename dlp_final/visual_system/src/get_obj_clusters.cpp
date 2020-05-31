#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>

/*
 * Parameters:
 *   [int] min_point_num*
 *   [int] max_point_num*
 *   [double] z_lower*
 *   [double] leaf_size*
 *   [double] min_cluster_dis*
 *   [string] arm_prefix
 * [Note] parameter with `*` can be changed in runtime
 */

/*
 * XXX
 * [Note] Minimum Euclidean distance to split objects: > 1.5 cm
 */

inline Eigen::Matrix4f tf2eigen(tf::Transform t){
  Eigen::Matrix4f res;
  tf::Matrix3x3 rot_mat(t.getRotation());
  res(0, 0) = rot_mat[0][0]; res(0, 1) = rot_mat[0][1]; res(0, 2) = rot_mat[0][2]; 
  res(1, 0) = rot_mat[1][0]; res(1, 1) = rot_mat[1][1]; res(1, 2) = rot_mat[1][2]; 
  res(2, 0) = rot_mat[2][0]; res(2, 1) = rot_mat[2][1]; res(2, 2) = rot_mat[2][2];
  res(0, 3) = t.getOrigin().getX();
  res(1, 3) = t.getOrigin().getY(); 
  res(2, 3) = t.getOrigin().getZ(); 
  res(3, 3) = 1.0f;
  return res;
}

class ObjClusters{
 private:
  int min_point_num, max_point_num;
  double z_lower, leaf_size, min_cluster_dis;
  const double Z_UPPER = 0.2;
  std::string arm_prefix, pc_frame, package_path;
  std::vector<std::string> pcd_path_vec;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_clusters;
  ros::Subscriber sub_pc;
  ros::ServiceServer get_clusters_srv;
  ros::Timer check_parameter_timer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr item_1_ptr, item_2_ptr, item_3_ptr;
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::VoxelGrid<pcl::PointXYZ> vg_model;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  tf::Transform getTransform(std::string target, std::string source, bool &result){
    tf::StampedTransform stf;
    static tf::TransformListener listener;
    try{
      listener.waitForTransform(target, source, ros::Time(0), ros::Duration(0.5));
      listener.lookupTransform(target, source, ros::Time(0), stf); result = true;
    } catch(tf::TransformException &ex){
      ROS_WARN("[%s] Can't get transform from [%s] to [%s]", \
                ros::this_node::getName().c_str(),
                target.c_str(),
                source.c_str()); result = false;
    }
    return (tf::Transform(stf.getRotation(), stf.getOrigin()));
  }
  void timer_cb(const ros::TimerEvent& event){
    int i_tmp;
    pnh_.getParam("min_point_num", i_tmp);
    if(i_tmp!=min_point_num){
      ROS_INFO("[%s] min_point_num set from %d to %d", ros::this_node::getName().c_str(), min_point_num, i_tmp);
      min_point_num = i_tmp;
    }
    pnh_.getParam("max_point_num", i_tmp);
    if(i_tmp!=max_point_num){
      ROS_INFO("[%s] max_point_num set from %d to %d", ros::this_node::getName().c_str(), max_point_num, i_tmp);
      max_point_num = i_tmp;
    }
    double d_tmp;
    pnh_.getParam("z_lower", d_tmp);
    if(d_tmp!=z_lower){
      ROS_INFO("[%s] z_lower set from %f to %f", ros::this_node::getName().c_str(), z_lower, d_tmp);
      z_lower = d_tmp;
    }
    pnh_.getParam("leaf_size", d_tmp);
    if(d_tmp!=leaf_size){
      ROS_INFO("[%s] leaf_size set from %f to %f", ros::this_node::getName().c_str(), leaf_size, d_tmp);
      leaf_size = d_tmp;
      vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    }
    /*
    pnh_.getParam("min_cluster_dis", d_tmp);
    if(d_tmp!=min_cluster_dis){
      ROS_INFO("[%s] min_cluster_dis set from %f to %f", ros::this_node::getName().c_str(), min_cluster_dis, d_tmp);
      min_cluster_dis = d_tmp;
    }
    */
  }
  void sub_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    if(pc_frame.empty()) pc_frame = msg->header.frame_id;
    pcl::fromROSMsg(*msg, *pc_ptr);
  }
  pcl::PointCloud<pcl::PointXYZRGB> segmentation(void){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed (new pcl::PointCloud<pcl::PointXYZRGB>()), 
                                           pub_pc (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // Remove NaN points
    #ifdef VERBOSE
      std::cout << "Raw data: " << pc_ptr->points.size() << "\n";
    #endif
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*pc_ptr, *processed, mapping);
    #ifdef VERBOSE
      std::cout << "Remove nan: " << processed->points.size() << "\n";
    #endif
    // Change frame
    bool can_get_transform;
    Eigen::Matrix4f transform_mat = tf2eigen(getTransform(arm_prefix+"base_link", pc_frame, can_get_transform));
    if(!can_get_transform){
      return *pub_pc; // Empty cloud
    }
    pcl::transformPointCloud(*processed, *processed, transform_mat);
    // Conditional removal
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, z_lower)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, Z_UPPER)));
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(processed);
    //condrem.setKeepOrganized(true);
    condrem.filter(*processed);
    // Conditional removal with keeping organized will set outliers to NaN, so we have to remove them again
    //pcl::removeNaNFromPointCloud(*processed, *processed, mapping);
    #ifdef VERBOSE
      std::cout << "Conditional removal: " << processed->points.size() << "\n";
    #endif
    // Downsampling
    vg.setInputCloud(processed);
    vg.filter(*processed);
    #ifdef VERBOSE
      std::cout << "Downsampling: " << processed->points.size() << "\n";
    #endif
    // Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(processed);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(min_cluster_dis);
    ec.setMinClusterSize(min_point_num);
    ec.setMaxClusterSize(max_point_num);
    ec.setSearchMethod(tree);
    ec.setInputCloud(processed);
    ec.extract(cluster_indices);
    for(int i=0; i<cluster_indices.size(); ++i){ // With points number order
      pcl::PointCloud<pcl::PointXYZRGB> cluster_pc(*processed, cluster_indices[i].indices);
      *pub_pc += cluster_pc;
    }
    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(*pub_pc, pc_out);
    pc_out.header.frame_id = arm_prefix+"base_link";
    pub_clusters.publish(pc_out);
    ROS_INFO("[%s] Detect %d clusters.", ros::this_node::getName().c_str(), (int)cluster_indices.size());
    std::stringstream ss;
    for(int i=0; i<cluster_indices.size(); ++i){
      ss << "Cluster " << i << " with " << cluster_indices[i].indices.size() << " points\n";
    }
    std::cout << ss.str();
    pcl::PointCloud<pcl::PointXYZRGB> most_important_pc(*processed, cluster_indices[0].indices);
    return most_important_pc;
  }
  bool srv_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("[%s] Receive new request, start processing...", ros::this_node::getName().c_str());
    if(pc_ptr->points.size()==0){
      ROS_ERROR("[%s] No points received yet, make sure you connect topic correctly!", ros::this_node::getName().c_str());
      return true;
    }
    auto target_pc = segmentation();
    pcl::PointCloud<pcl::PointXYZ> target_pc_xyz;
    // Can't use copyPointCloud from xyzrgb to xyz, so using for
    for(auto p_: target_pc.points){
      pcl::PointXYZ p;
      p.x = p_.x;
      p.y = p_.y;
      p.z = p_.z;
      target_pc_xyz.points.push_back(p);
    } target_pc_xyz.width = 1; target_pc_xyz.height = target_pc.points.size();
    if(target_pc.points.size()==0){
      ROS_ERROR("[%s] Can't get transform, abort request...", ros::this_node::getName().c_str());
      return true;
    }
    // ICP
    ROS_INFO("[%s] Start point registration...", ros::this_node::getName().c_str());
    double min_score = 1e6; int min_idx = -1;
    std::vector<Eigen::Matrix4f> res_mat_vec; res_mat_vec.resize(3);
    icp.setInputTarget(target_pc_xyz.makeShared());
    for(int i=0; i<3; ++i){
      switch(i){
        case 0:
          icp.setInputSource(item_1_ptr);
          break;
        case 1:
          icp.setInputSource(item_2_ptr);
          break;
        case 2:
          icp.setInputSource(item_3_ptr);
          break;
      }
      pcl::PointCloud<pcl::PointXYZ> registration_res;
      icp.align(registration_res);
      res_mat_vec[i] = icp.getFinalTransformation();
      if(min_score>icp.getFitnessScore()){
        min_score = icp.getFitnessScore();
        min_idx = i;
      }
    }
    std::string item;
    if(min_idx==0) item = "big_transformer";
    else if(min_idx==1) item = "buzzer";
    else if(min_idx==2) item = "mid_transformer";
    ROS_INFO("[%s] Done ICP, candidate item: %s with score: %f", ros::this_node::getName().c_str(), item.c_str(), min_score);
    return true;
  }
 public:
  // Constructor
  ObjClusters(ros::NodeHandle nh, ros::NodeHandle pnh): min_cluster_dis(0.01f), pc_frame(""), nh_(nh), pnh_(pnh){
    pcd_path_vec.resize(3);
    pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    item_1_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    item_2_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    item_3_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    package_path = ros::package::getPath("visual_system");
    // 0: big transformer, 1: buzzer, 2: mid transformer
    pcd_path_vec[0] = package_path + "/pcd/big_transformer_simplify.pcd";
    pcd_path_vec[1] = package_path + "/pcd/Buzzer_simplify.pcd";
    pcd_path_vec[2] = package_path + "/pcd/mid_transformer_simplify.pcd"; 
    if(pcl::io::loadPCDFile(pcd_path_vec[0], *item_1_ptr) == -1 or
       pcl::io::loadPCDFile(pcd_path_vec[1], *item_2_ptr) == -1 or
       pcl::io::loadPCDFile(pcd_path_vec[2], *item_3_ptr) == -1){
      ROS_ERROR("Can't find PCD file, exit...");
      ros::shutdown();
    }
    // Get parameters
    if(!pnh_.getParam("arm_prefix", arm_prefix)) arm_prefix="";
    /*
    if(!pnh_.getParam("min_cluster_dis", min_cluster_dis)){
      min_cluster_dis = 0.02f;
      ROS_WARN("[%s] min_cluster_dis not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("min_cluster_dis", min_cluster_dis);
    }
    */
    if(!pnh_.getParam("z_lower", z_lower)){
      z_lower = 0.01f;
      ROS_WARN("[%s] z_lower not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("z_lower", z_lower);
    }
    if(!pnh_.getParam("leaf_size", leaf_size)){
      leaf_size = 0.05f;
      ROS_WARN("[%s] leaf_size not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("leaf_size", leaf_size);
    }
    if(!pnh_.getParam("min_point_num", min_point_num)){
      min_point_num = 50;
      ROS_WARN("[%s] min_point_num not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("min_point_num", min_point_num);
    }
    if(!pnh_.getParam("max_point_num", max_point_num)){
      max_point_num = 500;
      ROS_WARN("[%s] max_point_num not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("max_point_num", max_point_num);
    }
    // Show parameters
    ROS_INFO("[%s]\n============================================\n\
min_point_num: %d\n\
max_point_num: %d\n\
arm_prefix: %s\n\
z_lower: %f\n\
leaf_size: %f", ros::this_node::getName().c_str(), min_point_num, max_point_num, arm_prefix.c_str(), z_lower, leaf_size);
    #ifdef VERBOSE
      ROS_WARN("[%s] VERBOSE mode", ros::this_node::getName().c_str());
    #endif
    if(!arm_prefix.empty()) arm_prefix+="_";
    // Downsampling
    vg_model.setLeafSize(0.005f, 0.005f, 0.005f); // For pre-saved models
    std::stringstream info_ss;
    info_ss << "\nItem 1: " << item_1_ptr->points.size() << " ";
    vg_model.setInputCloud(item_1_ptr);
    vg_model.filter(*item_1_ptr);
    info_ss << " -> " << item_1_ptr->points.size() << "\n"
            << "Item 2: " << item_2_ptr->points.size() << " ";
    vg_model.setInputCloud(item_2_ptr);
    vg_model.filter(*item_2_ptr);
    info_ss << " -> " << item_2_ptr->points.size() << "\n"
            << "Item 3: " << item_3_ptr->points.size() << " ";
    vg_model.setInputCloud(item_3_ptr);
    vg_model.filter(*item_3_ptr);
    info_ss << " -> " << item_3_ptr->points.size() << "\n";
    ROS_INFO("[%s] Successfully load models: %s", ros::this_node::getName().c_str(), info_ss.str().c_str());
    vg_model.setLeafSize(leaf_size, leaf_size, leaf_size);
    // Set ICP object
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-2);
    icp.setEuclideanFitnessEpsilon(1e-1);
    // Set timer, publisher, subscriber and service
    check_parameter_timer = pnh_.createTimer(ros::Duration(1.0), &ObjClusters::timer_cb, this);
    pub_clusters = pnh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);
    sub_pc = pnh_.subscribe("pc", 1, &ObjClusters::sub_cb, this);
    get_clusters_srv = pnh_.advertiseService("get_pose", &ObjClusters::srv_cb, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_pick_pose");
  ros::NodeHandle nh, pnh("~");
  ObjClusters foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
