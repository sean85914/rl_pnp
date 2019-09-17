#include <cv_bridge/cv_bridge.h> // cv::Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h> // pcl::PointXYZRGB
#include <pcl/common/projection_matrix.h> // pcl::PointCloud
#include <pcl/features/normal_3d_omp.h> // pcl::Norma;Estimation

cv::Mat applyMeanFilter(cv::Mat original_image, int kernel_size);
bool isPointInCloud(const pcl::PointCloud<pcl::PointXYZRGB> pc, const pcl::PointXYZRGB p);
bool isPointInCloud(const pcl::PointCloud<pcl::PointXYZ> pc, const pcl::PointXYZ p);
Eigen::Vector4f getCentroidPoint(const double radius, 
                                 const pcl::PointCloud<pcl::PointXYZ> pc, 
                                 const pcl::PointXYZ p);
Eigen::Vector3f getSurfaceNormal(const double radius, 
                                 const pcl::PointCloud<pcl::PointXYZ> pc,
                                 const pcl::PointXYZ p);
