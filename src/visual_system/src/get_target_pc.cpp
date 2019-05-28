#include <ros/ros.h>
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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

class GetTargetPC{
 private:
  bool type; // Draw type: true -> sphere, false -> arrow
  const double DIS_CAM2CONVEYOR = 0.65;
  PointCloudXYZRGBPtr inputPtr;
  PointCloudXYZRGBPtr targetPtr;
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pc;
  ros::Publisher pub_pc;
  ros::Publisher pub_marker;
  visualization_msgs::Marker marker;
  void cb_pc(const sensor_msgs::PointCloud2ConstPtr);
 public:
  GetTargetPC(ros::NodeHandle, ros::NodeHandle);
};

GetTargetPC::GetTargetPC(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
  sub_pc = pnh_.subscribe("pointcloud", 1, &GetTargetPC::cb_pc, this);
  pub_pc = pnh_.advertise<sensor_msgs::PointCloud2>("target_pc", 1);
  pub_marker = pnh_.advertise<visualization_msgs::Marker>("centroid", 1);
  if(!pnh_.getParam("type", type)) type = true; ROS_INFO("type: %s", (type==true?"true":"false")); 
  inputPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  targetPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.g = marker.color.a = 1.0f; // green
  marker.pose.orientation.w = 1.0f;
  if(type){
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.02f;
  }
  else{
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.01f; marker.scale.y = 0.012f;
    marker.scale.z = 0.0f;
  }
}

void GetTargetPC::cb_pc(const sensor_msgs::PointCloud2ConstPtr msgPtr){
  marker.points.clear();
  pcl::fromROSMsg(*msgPtr, *inputPtr);
  // RemoveNaN
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
  if(type){ // Sphere
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
  }else{ // Arrow
    // Build pointcloud which is the K nearest neighbor of centroid
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
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (pointIdxRadiusSearch)); 
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
    // Get centroid point index
    int idx = 0;
    for(size_t i=0; i<centroid_cloud->points.size(); ++i){
      if(centroid_cloud->points[i].x == searchPoint.x &&
         centroid_cloud->points[i].y == searchPoint.y &&
         centroid_cloud->points[i].z == searchPoint.z ) {idx = i; ROS_INFO("find"); break;}
    }
    sensor_msgs::PointCloud2 pcout;
    pcl::toROSMsg(*targetPtr, pcout);
    pcout.header.frame_id = msgPtr->header.frame_id;
    pub_pc.publish(pcout);
    geometry_msgs::Point p;
    p.x = centroid[0]; p.y = centroid[1]; p.z = centroid[2];
    marker.points.push_back(p);
    p.x += cloud_normals->points[idx].normal[0]*0.1f;
    p.y += cloud_normals->points[idx].normal[1]*0.1f;
    p.z += cloud_normals->points[idx].normal[2]*0.1f;
    marker.points.push_back(p);
  }
  marker.header.frame_id = msgPtr->header.frame_id;
  pub_marker.publish(marker);
  sensor_msgs::PointCloud2 pcout;
  pcl::toROSMsg(*targetPtr, pcout);
  pcout.header.frame_id = msgPtr->header.frame_id;
  pub_pc.publish(pcout);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_target_pc_node");
  ros::NodeHandle nh, pnh("~");
  GetTargetPC foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
