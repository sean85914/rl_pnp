#include "visual_system/helper.h"

cv::Mat applyMeanFilter(cv::Mat original_image, int kernel_size){
  int l = floor(double(kernel_size)/2);
  int count = 0;
  const int ROW = original_image.rows;
  const int COL = original_image.cols;
  cv::Mat dst = original_image.clone();
  for(int x=0; x<COL; ++x){
    for(int y=0; y<ROW; ++y){
      if(original_image.at<unsigned short>(cv::Point(x, y))!=0)
        continue;
      int cnt = 0;
      unsigned short val = 0;
      for(int i=-l; i<=l; ++i){
        for(int j=-l; j<=l; ++j){
          if(x+i<l or x+i>=COL or y+j<l or y+j>=ROW)
            continue;
          if(original_image.at<unsigned short>(cv::Point(x+i, y+j))!=0){
            val += original_image.at<unsigned short>(cv::Point(x+i, y+j)); ++cnt;
          } // end if
        } // end for (j)
      } // end for (i)
      if(cnt!=0) {
        count++;
        unsigned short new_val = double(val)/cnt;
        dst.at<unsigned short>(cv::Point(x, y)) = new_val;
      }
    } // end for (y) 
  } // end for (x)
  return dst;
}

bool isPointInCloud(const pcl::PointCloud<pcl::PointXYZRGB> pc, const pcl::PointXYZRGB p){
  for(auto p_in_pc: pc.points){
    if(p_in_pc.x==p.x and p_in_pc.y==p.y and p_in_pc.z==p.z) return true;
  } return false;
}

bool isPointInCloud(const pcl::PointCloud<pcl::PointXYZ> pc, const pcl::PointXYZ p){
  for(auto p_in_pc: pc.points){
    if(p_in_pc.x==p.x and p_in_pc.y==p.y and p_in_pc.z==p.z) return true;
  } return false;
}

Eigen::Vector4f getCentroidPoint(const double radius,
                                 const pcl::PointCloud<pcl::PointXYZ> pc, 
                                 const pcl::PointXYZ p){
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree.setInputCloud(pc.makeShared());
  kdtree.radiusSearch(p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  pcl::PointCloud<pcl::PointXYZ> pc_in_radius(pc, pointIdxRadiusSearch);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(pc_in_radius, centroid);
  return centroid;
}

Eigen::Vector3f getSurfaceNormal(const double radius, 
                                 const pcl::PointCloud<pcl::PointXYZ> pc,
                                 const pcl::PointXYZ p){
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(pc.makeShared());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
  ne.setRadiusSearch(radius);
  ne.compute(*cloud_normals);
  int idx = -1;
  for(int i=0; i<cloud_normals->points.size(); ++i){
    if(pc.points[i].x == p.x and pc.points[i].y == p.y and pc.points[i].z == p.z) idx = i;
  }
  Eigen::Vector3f normal;
  normal(0) = normal(1) = normal(2) = 0.0f;
  // Use the specific point normal
  /*if(idx == -1){
    std::cout << "\033[1;31mNot found specific point in the point cloud set\033[0m\n";
    return normal;
  }
  normal(0) = cloud_normals->points[idx].normal_x;
  normal(1) = cloud_normals->points[idx].normal_y;
  normal(2) = cloud_normals->points[idx].normal_z;*/
  // Use all points in the disk
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree.setInputCloud(pc.makeShared());
  kdtree.radiusSearch(p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  int num = pointIdxRadiusSearch.size();
  for(auto idx: pointIdxRadiusSearch){
    if(std::isnan(cloud_normals->points[idx].normal_x) or std::isnan(cloud_normals->points[idx].normal_y) or std::isnan(cloud_normals->points[idx].normal_z)) {--num; continue;}
    normal(0) += cloud_normals->points[idx].normal_x;
    normal(1) += cloud_normals->points[idx].normal_y;
    normal(2) += cloud_normals->points[idx].normal_z;
  } normal = normal / (double)num;
  normal.normalize();
  if(normal(2)>0.0f) normal *= -1.0f; // Makes it down toward the table
  return normal;
} 
