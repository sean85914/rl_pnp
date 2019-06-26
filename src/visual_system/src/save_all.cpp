#include <fstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

std::fstream f_intrisic, f_pc;
std::vector<double> intrinsic;
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr;

void callback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info, const sensor_msgs::PointCloud2ConstPtr& pc)
{
  intrinsic[0] = cam_info->K[0];
  intrinsic[1] = cam_info->K[4];
  intrinsic[2] = cam_info->K[2];
  intrinsic[3] = cam_info->K[5];
  try{
    color_img_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }  
  try{
    depth_img_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
  pcl::fromROSMsg(*pc, *pc_ptr);
}

bool cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  static int counter = 0;
  std::string color_file_name = std::to_string(counter) + ".jpg";
  std::string depth_file_name = std::to_string(counter) + ".png";
  std::string pc_file_name = std::to_string(counter) + ".pcd";
  cv::imwrite(color_file_name, color_img_ptr->image);
  cv::imwrite(depth_file_name, depth_img_ptr->image);
  pcl::io::savePCDFileASCII(pc_file_name, *pc_ptr);
  f_intrisic << intrinsic[0] << " "
             << intrinsic[1] << " "
             << intrinsic[2] << " "
             << intrinsic[3] << "\n";
  for(int col=0; col<pc_ptr->width; ++col){
    for(int row=0; row<pc_ptr->height; ++row){
      f_pc << col << " "
           << row << " "
           << pc_ptr->at(col, row).x << " "
           << pc_ptr->at(col, row).y << " "
           << pc_ptr->at(col, row).z << "\n";
    }
  }
}

int main(int argc, char** argv)
{
  f_intrisic.open("test.txt", std::ios::out);
  f_pc.open("pc.txt", std::ios::out);
  intrinsic.resize(4);
  pc_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  ros::init(argc, argv, "save_all");
  ros::NodeHandle nh, pnh("~");
  ros::ServiceServer save_all_server = pnh.advertiseService("save_all", cb);
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera/color/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "camera/depth_registered/points", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, \
                                   sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, \
                                                  info_sub, pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  ros::spin();
  f_intrisic.close(); f_pc.close();
  return 0;
}
