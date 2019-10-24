#include <sstream>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <visual_system/charuco_detector.h>

bool has_transform = false;
const int ROW = 6;
const int COL = 3;
const int NUM = 4;
const int BITS = 50;
const double SQUARE_LEN = 0.04f;
const double TAG_LEN = 0.032f;

CharucoDetector detector1, detector2;
tf::Transform cam1_to_cam2;

tf::Vector3 point2tf(const geometry_msgs::Point p){
  return (tf::Vector3(p.x, p.y, p.z));
}

double compute_squared_distance(const tf::Vector3 p1, const tf::Vector3 p2){
  double dx = p1.getX() - p2.getX();
  double dy = p1.getY() - p2.getY();
  double dz = p1.getZ() - p2.getZ();
  return (dx*dx+dy*dy+dz*dz);
}

tf::Transform getTransform(std::string source, std::string target){
  static tf::TransformListener listener;
  tf::StampedTransform stf;
  try{
    listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1.0f));
    listener.lookupTransform(source, target, ros::Time(0), stf);
    tf::Transform tf(stf.getRotation(), stf.getOrigin());
    return tf;
  } catch(tf::TransformException& e){
    ROS_ERROR("%s", e.what());
    exit(EXIT_FAILURE);
  }
}

void callback(const sensor_msgs::ImageConstPtr& image1,
              const sensor_msgs::CameraInfoConstPtr& info1,
              const sensor_msgs::ImageConstPtr& image2,
              const sensor_msgs::CameraInfoConstPtr& info2){
  if(!has_transform) cam1_to_cam2 = getTransform(image1->header.frame_id, image2->header.frame_id);
  detector1.setIntrinsic(info1->K[0], info1->K[4], info1->K[2], info1->K[5]);
  detector2.setIntrinsic(info2->K[0], info2->K[4], info2->K[2], info2->K[5]);
  cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
  try{
    cv_ptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    cv_ptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  detector1.setImage(cv_ptr1->image);
  detector2.setImage(cv_ptr2->image);
  cv::Vec3d rvec1, rvec2, tvec1, tvec2;
  cv::Mat _;
  auto res_map_1 = detector1.getCornersPosition(_, rvec1, tvec1);
  auto res_map_2 = detector2.getCornersPosition(_, rvec2, tvec2);
  int cnt = 0;
  double mean_squared_distance = 0.0f;
  std::stringstream ss;
  for(auto element: res_map_1){
    std::map<int, geometry_msgs::Point>::iterator it;
    it = res_map_2.find(element.first);
    if(it!=res_map_2.end()){
      ss << std::to_string(element.first) << " ";
      ++cnt;
      auto corner_in_1 = point2tf(element.second);
      auto corner_in_2 = point2tf(it->second);
      auto corner_2_to_1 = cam1_to_cam2*corner_in_2;
      mean_squared_distance += compute_squared_distance(corner_in_1, corner_2_to_1);
    }
  } ss << "\n";
  if(cnt==0) {ROS_WARN("No common corner detected"); return;}
  double mm_error = sqrt(mean_squared_distance/cnt)*1000;
  ss << std::to_string(mm_error);
  ROS_INFO("%s", ss.str().c_str());
}

int main(int argc, char** argv)
{
  detector1 = CharucoDetector(ROW, COL, NUM, BITS, SQUARE_LEN, TAG_LEN);
  detector2 = CharucoDetector(ROW, COL, NUM, BITS, SQUARE_LEN, TAG_LEN);
  ros::init(argc, argv, "validate_calibration");
  ros::NodeHandle nh, pnh("~");
  message_filters::Subscriber<sensor_msgs::Image> image1_sub(pnh, "image1", 1);
  message_filters::Subscriber<sensor_msgs::Image> image2_sub(pnh, "image2", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info1_sub(pnh, "info1", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info2_sub(pnh, "info2", 1);
  typedef message_filters::sync_policies::ApproximateTime\
    <sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> \
    MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> \
    sync(MySyncPolicy(10), image1_sub, info1_sub, image2_sub, info2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  while(ros::ok()) ros::spinOnce();
  return 0;
}
