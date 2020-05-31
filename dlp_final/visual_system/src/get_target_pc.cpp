#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h> // PointXYZ
#include <pcl/common/centroid.h> // compute3DCentroid
#include <pcl/kdtree/kdtree_flann.h> // KdTree
#include <pcl/features/normal_3d.h> // NormalEstimation
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
#include <pcl/filters/extract_indices.h> // Indice filter
// Message 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visual_system/target_pose.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

class GetTargetPC{
 private:
  bool spin; // if callback should publish data
  int pub_time; // marker publisher publish time
  const int PUB_TIMES = 10; // one service call should only publish marker PUB_TIMES times
  const double DIS_CAM2CONVEYOR = 0.65; // distance from camera to conveyor
  std::string camera_name; // camera namespace, from parameter server
  std::string topic_name; // pointcloud topic name = /\camera_name\/depth_registered/points
  std::string frame; // pointcloud frame id = /\camera_name\_color_optical_frame
  PointCloudXYZRGBPtr inputPtr; // cloud pointer for input
  PointCloudXYZRGBPtr targetPtr; // cloud pointer for target (after input remove z greater than DIS_CAM2CONVEYOR)
  visualization_msgs::Marker marker;
  tf::TransformListener listener;
  ros::NodeHandle nh_, pnh_;
  ros::Timer timer;
  ros::Subscriber sub_pc;
  ros::Publisher pub_marker;
  ros::ServiceServer getTargetInfo_srv;
  bool cb_service(visual_system::target_pose::Request&, visual_system::target_pose::Response&);
  void cb_timer(const ros::TimerEvent&);
 public:
  GetTargetPC(ros::NodeHandle, ros::NodeHandle);
};

GetTargetPC::GetTargetPC(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), spin(false), pub_time(0){
  getTargetInfo_srv = pnh_.advertiseService("get_target_info", &GetTargetPC::cb_service, this);
  timer = pnh_.createTimer(ros::Duration(3.0), &GetTargetPC::cb_timer, this);
  pub_marker = pnh_.advertise<visualization_msgs::Marker>("normal_marker", 1);
  if(!pnh_.getParam("camera_name", camera_name)) camera_name = "camera";
  topic_name = "/" + camera_name + "/depth_registered/points"; ROS_INFO("pointcloud topic: %s", topic_name.c_str());
  frame = camera_name + "_color_optical_frame";
  inputPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  targetPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.g = marker.color.a = 1.0f;
  marker.scale.x = 0.01f; marker.scale.y = 0.012f; marker.scale.z = 0.0f;
  marker.pose.orientation.w = 1.0f;
}

void GetTargetPC::cb_timer(const ros::TimerEvent& event){
  if(!spin) return;
  if(pub_time == PUB_TIMES){ pub_time = 0; spin = false; return;}
  pub_marker.publish(marker); ++pub_time;
}

bool GetTargetPC::cb_service(visual_system::target_pose::Request &req, visual_system::target_pose::Response &res){
  ROS_INFO("Service called, start processing...");
  sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_name, ros::Duration(3.0));
  if(pc == NULL) {ROS_ERROR("No pointcloud received."); return false;}
  spin = true; marker.points.clear();
  // Convert to pcl and remove nan points
  pcl::fromROSMsg(*pc, *inputPtr);
  std::vector<int> _;
  pcl::removeNaNFromPointCloud(*inputPtr, *inputPtr, _);
  // Remove z that close to the plane
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(inputPtr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, DIS_CAM2CONVEYOR);
  pass.filter(*targetPtr);
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*targetPtr, centroid);
  ROS_INFO("There are %d points remain, with centroid (%f, %f, %f).",
    (int)targetPtr->points.size(), centroid[0], centroid[1], centroid[2]);
  // Build pointcloud around centroid
  PointCloudXYZRGBPtr centroid_cloud (new PointCloudXYZRGB ());
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(targetPtr);
  pcl::PointXYZRGB searchPoint; 
  searchPoint.x = centroid[0]; searchPoint.y = centroid[1]; searchPoint.z = centroid[2];
  double radius = 0.03f;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  ROS_INFO("%d points within radius", (int)pointIdxRadiusSearch.size());
  pcl::ExtractIndices<pcl::PointXYZRGB> indiceFilter(true);
  // Convert std::vector<int> to pcl::PointIndices
  boost::shared_ptr<std::vector<int>> indicesptr (new std::vector<int> (pointIdxRadiusSearch)); 
  indiceFilter.setInputCloud(targetPtr);
  indiceFilter.setIndices(indicesptr);
  indiceFilter.setNegative(false);
  indiceFilter.filter(*centroid_cloud);
  // Compute normal of centroid
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(centroid_cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.02f);
  ne.compute(*cloud_normals);
  ROS_INFO("Get transform from [%s] to [base_link]", frame.c_str());
  tf::StampedTransform st;
  try{
    listener.waitForTransform("base_link", frame, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("base_link", frame, ros::Time(0), st);
  } catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }
  tf::Vector3 origin = st.getOrigin();
  tf::Quaternion orientation = st.getRotation();
  tf::Matrix3x3 rot_mat(orientation);
  tf::Transform transform(orientation, origin);
  tf::Vector3 centroid_tf(centroid[0], centroid[1], centroid[2]),
              normal_tf(cloud_normals->points[0].normal[0],
                        cloud_normals->points[0].normal[1],
                        cloud_normals->points[0].normal[2]);
  tf::Vector3 centroid_transformed = transform*centroid_tf,
              normal_transformed;
  normal_transformed.setX(rot_mat[0].dot(normal_tf));
  normal_transformed.setY(rot_mat[1].dot(normal_tf));
  normal_transformed.setZ(rot_mat[2].dot(normal_tf));
  normal_transformed.normalize();
  res.centroid.x = centroid_transformed.getX(); res.normal.x = normal_transformed.getX();
  res.centroid.y = centroid_transformed.getY(); res.normal.y = normal_transformed.getY();
  res.centroid.z = centroid_transformed.getZ(); res.normal.z = normal_transformed.getZ();
  geometry_msgs::Point p;
  p = res.centroid;
  marker.points.push_back(p);
  p.x += normal_transformed.getX()*0.05;
  p.y += normal_transformed.getY()*0.05;
  p.z += normal_transformed.getZ()*0.05;
  marker.points.push_back(p);
  marker.header.frame_id = "base_link";
 /* p.x = centroid[0]; p.y = centroid[1]; p.z = centroid[2]; 
  marker.points.push_back(p);
  p.x += cloud_normals->points[0].normal[0]*0.05;
  p.y += cloud_normals->points[0].normal[1]*0.05;
  p.z += cloud_normals->points[0].normal[2]*0.05;
  marker.points.push_back(p);
  marker.header.frame_id = pc->header.frame_id;*/
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_target_pc_node");
  ros::NodeHandle nh, pnh("~");
  GetTargetPC foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
