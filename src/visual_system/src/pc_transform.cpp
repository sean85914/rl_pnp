#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp> // cv
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> // savePCDFileASCII
#include <pcl/point_types.h> // PointXYZ
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/common/transforms.h> // transformPointCloud
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/SetBool.h>
#include <visual_system/get_xyz.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

bool verbose;
int num_thres;
const int LENGTH = 224; // deprecated
const int X_MIN = 246; // deprecated
const int Y_MIN = 93; // deprecated
const double X_LOWER = -0.659f;
const double X_UPPER = -0.273f;
const double Y_LOWER = -0.269f;
const double Y_UPPER =  0.117f;
const double Z_LOWER = -0.01f;
const double Z_UPPER =  0.20f;
const double Z_THRES_LOWER = 0.01f;
const double Z_THRES_UPPER = 0.20f;
std::string frame_name;
std::vector<double> intrinsic; // [fx, fy, cx, cy]
cv::Rect myROI(X_MIN, Y_MIN, LENGTH, LENGTH); // deprecated
cv::Mat last_color, last_depth;
cv_bridge::CvImagePtr color_img_ptr, depth_img_ptr;
Eigen::Matrix4f arm2cam_tf;
pcl::PointCloud<pcl::PointXYZRGB> pc;
pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
ros::Publisher pub_pc, pub_color, pub_depth, /*pub_marker, pub_text_marker*/;

Eigen::Matrix4f lookup_transform(void);
void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
                  const sensor_msgs::CameraInfoConstPtr& cam_info);
bool callback_service(visual_system::get_xyz::Request &req,
                      visual_system::get_xyz::Response &res);
bool callback_get_pc(visual_system::get_pc::Request &req,
                     visual_system::get_pc::Response &res);
//bool callback_is_empty(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res);

int main(int argc, char** argv)
{
  intrinsic.resize(4);
  ros::init(argc, argv, "pixel_to_xyz");
  ros::NodeHandle nh, pnh("~");
  if(!pnh.getParam("verbose", verbose)) verbose = false;
  if(!pnh.getParam("num_thres", num_thres)) num_thres = 7000; // Points number greater than this value will be considered as not empty
  if(verbose){
    ROS_WARN("Save pointcloud for debuging");
    std::string package_path = ros::package::getPath("grasp_suck");
    boost::filesystem::path p(package_path+"/pc");
    if(!boost::filesystem::exists(p))
      create_directories(p);
    //pub_pc = pnh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    //pub_color = pnh.advertise<sensor_msgs::Image>("crop_color", 1);
    //pub_depth = pnh.advertise<sensor_msgs::Image>("crop_depth", 1);
    //pub_marker = pnh.advertise<visualization_msgs::Marker>("marker", 1);
    //pub_text_marker = pnh.advertise<visualization_msgs::Marker>("text_marker", 1);
  }
  else
    ROS_WARN("Not save pointcloud");
  arm2cam_tf = lookup_transform();
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(X_LOWER, X_UPPER);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(Y_LOWER, Y_UPPER);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(Z_LOWER, Z_UPPER);
  // Message filter: color image, depth image, color camera info
  message_filters::Subscriber<sensor_msgs::Image> color_image_sub(nh, "camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera/color/camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, \
                                   sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub, depth_image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback_sub, _1, _2, _3));
  //ros::ServiceServer service = pnh.advertiseService("pixel_to_xyz", callback_service);
  //ros::ServiceServer image_service = pnh.advertiseService("get_image", callback_get_image);
  ros::ServiceServer pc_service = pnh.advertiseService("get_pc", callback_get_pc);
  ros::ServiceServer check_empty_service = pnh.advertiseService("empty_state", callback_is_empty);
  ros::spin();
  return 0;
}

Eigen::Matrix4f lookup_transform(void){
  Eigen::Matrix4f eigen_mat = Eigen::Matrix4f::Identity();
  tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform("base_link", "camera1_color_optical_frame", ros::Time(0), ros::Duration(0.3));
    listener.lookupTransform("base_link", "camera1_color_optical_frame", ros::Time(0), stf);
  } catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }
  tf::Matrix3x3 tf_mat(stf.getRotation());
  eigen_mat(0, 0) = tf_mat[0].getX(); eigen_mat(1, 0) = tf_mat[1].getX(); eigen_mat(2, 0) = tf_mat[2].getX(); 
  eigen_mat(0, 1) = tf_mat[0].getY(); eigen_mat(1, 1) = tf_mat[1].getY(); eigen_mat(2, 1) = tf_mat[2].getY(); 
  eigen_mat(0, 2) = tf_mat[0].getZ(); eigen_mat(1, 2) = tf_mat[1].getZ(); eigen_mat(2, 2) = tf_mat[2].getZ(); 
  eigen_mat(0, 3) = stf.getOrigin().getX(); eigen_mat(1, 3) = stf.getOrigin().getY(); eigen_mat(2, 3) = stf.getOrigin().getZ(); 
  return eigen_mat;
}

void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  frame_name = color_image->header.frame_id;
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
  pc.clear();
  for(int x=0; x<color_img_ptr->image.cols; ++x){ // 0~639
    for(int y=0; y<color_img_ptr->image.rows; ++y){ // 0~479
      auto depth = depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
      pcl::PointXYZRGB p;
      p.z = depth*0.001;
      p.x = (x-intrinsic[2])/intrinsic[0]*p.z;
      p.y = (y-intrinsic[3])/intrinsic[1]*p.z;
      p.r = color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
      p.g = color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
      p.b = color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
      pc.points.push_back(p);
    }
  }
}

/*void callback_sub(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image, \
              const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  frame_name = color_image->header.frame_id;
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
  pc.clear();
  for(int x=X_MIN; x<=X_MIN+LENGTH-1; ++x){ // 208~431
    for(int y=Y_MIN; y<=Y_MIN+LENGTH-1; ++y){ // 21~244
      // More readable
      auto depth = depth_img_ptr->image.at<unsigned short>(cv::Point(x, y));
      geometry_msgs::Point p;
      // Equivalent
      //if(depth!=0){
      //  pcl::PointXYZRGB p;
      //  p.z = depth * 0.001f;
      //  p.x = (x-intrinsic[2])/intrinsic[0]*p.z;
      //  p.y = (y-intrinsic[3])/intrinsic[1]*p.z;
      //  p.r = color_img_ptr->image.at<cv::Vec3b>(y, x)[2];
      //  p.g = color_img_ptr->image.at<cv::Vec3b>(y, x)[1];
      //  p.b = color_img_ptr->image.at<cv::Vec3b>(y, x)[0];
      //  pc.points.push_back(p);
      //}
      if(depth!=0){
        p.z = depth * 0.001f; // Represent in 1 mm
        p.x = (x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
        if(verbose){
          pcl::PointXYZRGB pc_p;
          pc_p.x = p.x; pc_p.y = p.y; pc_p.z = p.z;
          pc_p.r = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[2];
          pc_p.g = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[1];
          pc_p.b = color_img_ptr->image.at<cv::Vec3b>(cv::Point(x, y))[0];
          pc.push_back(pc_p);
        }
      } else{
        geometry_msgs::Point p;
        p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
      }
      pc.height = 1; pc.width = pc.points.size();
    } // End for(y)
  } // End for(x) 
  if(verbose){
    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(pc, pc_out);
    pc_out.header.frame_id = color_image->header.frame_id;
    pub_pc.publish(pc_out);
    cv::Mat crop_color = color_img_ptr->image(myROI), 
            crop_depth = depth_img_ptr->image(myROI);
    cv_bridge::CvImage crop_color_cv_bridge(color_img_ptr->header, color_img_ptr->encoding, crop_color),
                       crop_depth_cv_bridge(depth_img_ptr->header, depth_img_ptr->encoding, crop_depth);
    pub_color.publish(crop_color_cv_bridge.toImageMsg());
    pub_depth.publish(crop_depth_cv_bridge.toImageMsg());
  }
}*/

// FIXME
/*
bool callback_is_empty(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::PointCloud<pcl::PointXYZRGB> pc_filtered;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cp_ptr = pc.makeShared();
  pass.setInputCloud(pc_cp_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, Z_THRES);
  pass.filter(pc_filtered);
  //ROS_INFO("%d points after filtered.", (int)pc_filtered.points.size());
  if(pc_filtered.points.size()>num_thres) res.success = false; // Not empty
  else res.success = true; // Is empty
  return true;
}*/

bool callback_is_empty(visual_system::pc_is_empty::Request &req, visual_system::pc_is_empty::Response &res)
{
  pcl::PointCloud<pcl::PointXYZRGB> pc, pc_filtered;
  pcl::fromROSMsg(req.input_pc, pc);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(pc.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(Z_THRES_LOWER, Z_THRES_UPPER);
  pass.filter(pc_filtered);
  //ROS_INFO("%d -> %d", (int)pc.points.size(), (int)pc_filtered.points.size());
  if(pc_filtered.points.size()<num_thres) res.is_empty.data = true;
  else res.is_empty.data = false;
  return true;
}

/*
bool callback_service(visual_system::get_xyz::Request  &req,
                      visual_system::get_xyz::Response &res){
  geometry_msgs::Point p;
  int pixel_x = X_MIN+req.point[0],
      pixel_y = Y_MIN+req.point[1];
  auto depth = last_depth.at<unsigned short>(cv::Point(req.point[0], req.point[1]));
  if(depth==0){
    p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
  } else{
    p.z = depth * 0.001f; // 1mm
    p.x = (pixel_x-intrinsic[2])/intrinsic[0]*p.z; // x = (u-cx)/fx*z
    p.y = (pixel_y-intrinsic[3])/intrinsic[1]*p.z; // y = (v-cy)/fy*z
  }
  res.result = p;
  ROS_INFO("\nRequest: pixel(%d, %d)\n\
Response: point(%f, %f, %f)", 
            req.point[1], req.point[0], p.x, p.y, p.z);
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_name;
  marker.action = visualization_msgs::Marker::ADD; 
  marker.type = visualization_msgs::Marker::SPHERE; 
  marker.pose.position = res.result; 
  marker.pose.orientation.w = 1.0f;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.02f; 
  marker.color.g = marker.color.a = 1.0f;
  pub_marker.publish(marker);
  if(std::isnan(p.x) or p.z<=Z_THRES){
    visualization_msgs::Marker text_marker;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.header.frame_id = "base_link";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.pose.position.x = -0.7; 
    text_marker.scale.z = 0.06;
    text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0; // White text
    text_marker.text = "invalid";
    pub_text_marker.publish(text_marker);
  }
  return true;
}
*/

bool callback_get_pc(visual_system::get_pc::Request &req, 
                     visual_system::get_pc::Response &res)
{
  static int cnt = 0;
  ros::Time ts = ros::Time::now();
  // Transform points to hand coordinate
  pcl::transformPointCloud(pc, pc, arm2cam_tf);
  // Pass through XYZ
  pass_x.setInputCloud(pc.makeShared());
  pass_x.filter(pc);
  pass_y.setInputCloud(pc.makeShared());
  pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pc, pc2);
  pc2.header.frame_id = "base_link";
  res.pc = pc2;
  if(verbose){
    std::string package_path = ros::package::getPath("grasp_suck");
    std::string pc_name = package_path + "/pc/" + std::to_string(cnt) + ".pcd";
    pcl::io::savePCDFileASCII(pc_name, pc);
    ++cnt;
  }
  return true;
}

/*bool callback_get_image(visual_system::get_image::Request &req,
                        visual_system::get_image::Response &res){
  cv::Mat crop_color = color_img_ptr->image(myROI), 
          crop_depth = depth_img_ptr->image(myROI);
  last_color = crop_color; last_depth = crop_depth;
  cv_bridge::CvImage crop_color_cv_bridge(color_img_ptr->header, color_img_ptr->encoding, crop_color),
                     crop_depth_cv_bridge(depth_img_ptr->header, depth_img_ptr->encoding, crop_depth);
  res.crop_color_img = *crop_color_cv_bridge.toImageMsg();
  res.crop_depth_img = *crop_depth_cv_bridge.toImageMsg();
  return true;
}*/
