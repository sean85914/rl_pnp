#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h> // PointXYZ
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
#include <pcl/filters/extract_indices.h> // Indice filter
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

inline double norm(std::vector<double> vec){
  double square_sum = 0;
  for(std::vector<double>::iterator it=vec.begin(); it!=vec.end(); ++it){
    square_sum += (*it) * (*it);
  } return sqrt(square_sum);
}

class GetConveyorPlane{
 private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pc;
  ros::Publisher pub_plane_pc;
  PointCloudXYZRGBPtr inputPtr;
  PointCloudXYZRGBPtr filteredPtr;
  PointCloudXYZRGBPtr planePtr;
  void cb_pc(const sensor_msgs::PointCloud2ConstPtr);
 public:
  GetConveyorPlane(ros::NodeHandle, ros::NodeHandle);
};

GetConveyorPlane::GetConveyorPlane(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
  sub_pc = pnh_.subscribe("pointcloud", 1, &GetConveyorPlane::cb_pc, this);
  pub_plane_pc = pnh_.advertise<sensor_msgs::PointCloud2>("plane_pc", 1);
  inputPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  filteredPtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
  planePtr = PointCloudXYZRGBPtr (new PointCloudXYZRGB ());
}

void GetConveyorPlane::cb_pc(const sensor_msgs::PointCloud2ConstPtr msgPtr){
  pcl::fromROSMsg(*msgPtr, *inputPtr);
  std::vector<int> _;
  pcl::removeNaNFromPointCloud(*inputPtr, *inputPtr, _);
  // Get plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane;
  seg_plane.setOptimizeCoefficients(true);
  seg_plane.setModelType (pcl::SACMODEL_PLANE);
  seg_plane.setMethodType (pcl::SAC_RANSAC);
  seg_plane.setDistanceThreshold(0.01f);
  seg_plane.setInputCloud(inputPtr);
  seg_plane.segment(*inliers, *coefficients);
  ROS_INFO("There are %d ground inliers\n \
            Ground coefficeints: [%f] x + [%f] y + [%f] z = [%f]",
            (int)inliers->indices.size(),
            coefficients->values[0], coefficients->values[1], 
            coefficients->values[2], coefficients->values[3]*-1);
  std::vector<double> norm_vec{coefficients->values[0], coefficients->values[1], coefficients->values[2]};
  double distance_origin2plane = std::abs((coefficients->values[3])/norm(norm_vec));
  ROS_INFO("Distance from origin to plane: %f", distance_origin2plane);
  pcl::ExtractIndices<pcl::PointXYZRGB> indiceFilter(true);
  indiceFilter.setInputCloud(inputPtr);
  indiceFilter.setIndices(inliers);
  indiceFilter.setNegative(false);
  indiceFilter.filter(*planePtr);
  // Publish pointcloud
  sensor_msgs::PointCloud2 pcout;
  pcl::toROSMsg(*planePtr, pcout);
  pcout.header.frame_id = msgPtr->header.frame_id;
  pub_plane_pc.publish(pcout);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_conveyor_plane_node");
  ros::NodeHandle nh, pnh("~");
  GetConveyorPlane foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
