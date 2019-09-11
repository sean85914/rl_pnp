#include "helper.h"

cv::Mat applyMeanFilter(cv::Mat original_image, int kernel_size){
  int l = floor(double(kernel_size)/2);
  const int ROW = original_image.rows;
  const int COL = original_image.cols;
  cv::Mat dst;
  dst.create(ROW, COL, original_image.type());
  for(int x=0; x<COL; ++x){
    for(int y=0; y<ROW; ++y){
      if(original_image.at<unsigned short>(cv::Point(x, y))!=0){ 
        dst.at<unsigned short>(cv::Point(x, y)) = original_image.at<unsigned short>(cv::Point(x, y));
        continue;
      }
      int cnt = 0;
      unsigned short val = 0;
      for(int i=-l; i<=l; ++i){
        for(int j=-l; j<=l; ++j){
          if(x+i<l or x+i>=COL or y+j<l or y+j>=ROW){
            dst.at<unsigned short>(cv::Point(x, y)) = original_image.at<unsigned short>(cv::Point(x, y));
            continue;
          }
          if(original_image.at<unsigned short>(cv::Point(x+i, y+j))!=0){
            val += original_image.at<unsigned short>(cv::Point(x+i, y+j)); ++cnt;
          } // end if
        } // end for (j)
      } // end for (i)
      if(cnt!=0) {
        unsigned short new_val = double(val)/cnt;
        dst.at<unsigned short>(cv::Point(x, y)) = new_val;
      }
    } // end for (y) 
  } // end for (x)
  return dst;
}

bool isPointInCloud(pcl::PointCloud<pcl::PointXYZRGB> pc, pcl::PointXYZRGB p){
  for(auto p_in_pc: pc.points){
    if(p_in_pc.x==p.x and p_in_pc.y==p.y and p_in_pc.z==p.z) return true;
  } return false;
}

bool isPointInCloud(pcl::PointCloud<pcl::PointXYZ> pc, pcl::PointXYZ p){
  for(auto p_in_pc: pc.points){
    if(p_in_pc.x==p.x and p_in_pc.y==p.y and p_in_pc.z==p.z) return true;
  } return false;
}
