#include <fstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/Empty.h>
#include <visual_system/get_xyz.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp> // cv

bool isReady = false;
std::vector<double> intrinsic; // [fx, fy, cx, cy]
std::vector<geometry_msgs::Point> pixel_to_xyz; // Column-major ((0, 0), (0, 1), ..., (1, 0), (1, 1), ...(223, 223))
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
#ifdef PUB_PC
ros::Publisher pub_pc;
#endif

void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
                  const sensor_msgs::CameraInfoConstPtr& cam_info);
bool callback_service(visual_system::get_xyz::Request &req,
                      visual_system::get_xyz::Response &res);

int main(int argc, char** argv)
{
  intrinsic.resize(4); pixel_to_xyz.resize(224*224);
  ros::init(argc, argv, "pixel_to_xyz");
  ros::NodeHandle nh, pnh("~");
  #ifdef PUB_PC
  ROA_WARN("Publish pointcloud for debug");
  pub_pc = pnh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  #endif
  #ifndef PUB_PC
  ROS_WARN("Not publish poibtcloud");
  #endif
  // Message filter: color image, depth image, color camera info
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera/color/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, \
                                   sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback_sub, _1, _2, _3));
  ros::ServiceServer service = pnh.advertiseService("pixel_to_xyz", callback_service);
  ros::spin();
  return 0;
}


void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  intrinsic[0] = cam_info->K[0]; // fx
  intrinsic[1] = cam_info->K[4]; // fy
  intrinsic[2] = cam_info->K[2]; // cx
  intrinsic[3] = cam_info->K[5]; // cy
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
  int idx_cnt = 0;
  for(int x=208; /*row<depth_img_ptr->image.rows;*/x<=452; ++x){
    for(int y=21; /*col<depth_img_ptr->image.cols;*/ y<=245; ++y){
      // More readable
      auto depth = depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
      geometry_msgs::Point p;
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
      if(depth!=0){
        p.z = depth * 0.001f; // Represent in 1mm
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
        #ifdef PUB_PC
        pcl::PointXYZRGB pc_p;
        pc_p.x = p.x; pc_p.y = p.y; pc_p.z = p.z;
        pc_p.r = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[2];
        pc_p.g = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[1];
        pc_p.b = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[0];
        pc.push_back(pc_p);
        #endif
      } else{
        geometry_msgs::Point p;
        p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
      }
      pixel_to_xyz[idx_cnt] = p; ++idx_cnt;
      pc.height = 1; pc.width = pc.points.size();
      #ifdef PUB_PC
      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(pc, pc_out);
      pc_out.header.frmae_id = color_image->header.frame_id;
      pub_pc.publish(pc_out);
      #endif
    }
  } isReady = true;
}

bool callback_service(visual_system::get_xyz::Request &req,
                      visual_system::get_xyz::Response &res){
  if(!isReady){
    ROS_WARN("Receive request while not ready.");
    res.result.x = std::numeric_limits<double>::quiet_NaN();
    res.result.y = std::numeric_limits<double>::quiet_NaN();
    res.result.z = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  geometry_msgs::Point p = pixel_to_xyz[req.point[0]*224+req.point[1]+1];
  res.result = p;
  ROS_INFO("\nRequest: pixel(%d, %d)\n\
Response: point(%f, %f, %f)", 
              req.point[0], req.point[1], p.x, p.y, p.z);
  isReady = false; // Change to false 
  return true;
}
