#include <cv_bridge/cv_bridge.h> // cv::Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h> // pcl::PointXYZRGB
#include <pcl/common/projection_matrix.h> // pcl::PointCloud

cv::Mat applyMeanFilter(cv::Mat original_image, int kernel_size);
bool isPointInCloud(pcl::PointCloud<pcl::PointXYZRGB> pc, pcl::PointXYZRGB p);
bool isPointInCloud(pcl::PointCloud<pcl::PointXYZ> pc, pcl::PointXYZ p);
