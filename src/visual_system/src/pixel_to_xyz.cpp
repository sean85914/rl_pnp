#include <fstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

std::vector<double> intrinsic;
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
ros::Publisher pub_pc;

void callback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info)
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
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  for(int x=208; /*row<depth_img_ptr->image.rows;*/x<=452; ++x){
    for(int y=21; /*col<depth_img_ptr->image.cols;*/ y<=245; ++y){
      // More readable
      unsigned short int depth = depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
      if(depth!=0){
        pcl::PointXYZRGB p;
        p.z = depth * 0.001f;
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z;
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z;
        p.r = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[2];
        p.g = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[1];
        p.b = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[0];
        pc.points.push_back(p);
      }
    // Equivalent
    /*unsigned short int depth = depth_img_ptr->image.at<unsigned short>(y, x);
    if(depth!=0){
        pcl::PointXYZRGB p;
        p.z = depth * 0.001f;
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z;
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z;
        p.r = color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
        p.g = color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
        p.b = color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
        pc.points.push_back(p);
      }*/
    }
  } pc.height = 1; pc.width = pc.points.size();
  sensor_msgs::PointCloud2 pc_out;
  pcl::toROSMsg(pc, pc_out);
  pc_out.header.frame_id = "camera_color_optical_frame";
  pub_pc.publish(pc_out);
}

int main(int argc, char** argv)
{
  intrinsic.resize(4);
  ros::init(argc, argv, "generate_pointcloud");
  ros::NodeHandle nh, pnh("~");
  pub_pc = pnh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera/color/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, \
                                   sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  ros::spin();
  return 0;
}
