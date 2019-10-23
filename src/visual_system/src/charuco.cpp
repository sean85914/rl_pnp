#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <charuco_detector.h>
#include "conversion.hpp"


const int ROW = 6;
const int COL = 3;
const int NUM = 4;
const int BITS = 50;
const double SQUARE_LEN = 0.04f;
const double TAG_LEN = 0.032f;
CharucoDetector detector;
ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr &image, 
              const sensor_msgs::CameraInfoConstPtr &info){
  double fx = info->K[0];
  double fy = info->K[4];
  double cx = info->K[2];
  double cy = info->K[5];
  detector.setIntrinsic(fx, fy, cx, cy);
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat draw_img;
  try{
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  detector.setImage(cv_ptr->image);
  cv::Vec3d rvec, tvec;
  auto res_map = detector.getCornersPosition(draw_img, rvec, tvec);
  sensor_msgs::Image pub_image;
  cv_bridge::CvImage out_msg;
  out_msg.header.frame_id = image->header.frame_id;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = draw_img;
  pub.publish(out_msg);
  if(res_map.size()==0) return;
  auto rot_mat = rvec2tf(rvec);
  auto origin  = tvec2tf(tvec);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setBasis(rot_mat); transform.setOrigin(origin);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), image->header.frame_id, "charuco"));
  for(auto pair: res_map){
    int id = pair.first;
    geometry_msgs::Point p = pair.second;
    auto corner_origin = point2tf(p);
    tf::Transform corner_transform(transform);
    corner_transform.setOrigin(corner_origin);
    br.sendTransform(tf::StampedTransform(corner_transform, ros::Time::now(), image->header.frame_id, "tag_"+std::to_string(id)));
  }
  ros::Duration(1.0f/30.0f).sleep();
}

int main(int argc, char** argv)
{
  detector = CharucoDetector(ROW, COL, NUM, BITS, SQUARE_LEN, TAG_LEN);
  ros::init(argc, argv, "charuco_calibrator");
  ros::NodeHandle nh, pnh("~");
  message_filters::Subscriber<sensor_msgs::Image> image_sub(pnh, "image", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(pnh, "camera_info", 1);
  typedef message_filters::sync_policies::ApproximateTime\
                                        <sensor_msgs::Image, 
                                         sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> 
    sync(MySyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pub = pnh.advertise<sensor_msgs::Image>("processed", 1);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
