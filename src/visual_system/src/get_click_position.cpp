#include <ros/ros.h>
#include <tf/transform_listener.h>
// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Message type
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
// DEBUG: 
//   1. Visualize the result in RViz with sphere marker
//   2. Print execution time in ms  
#define DEBUG 0
#ifdef DEBUG
#include <visualization_msgs/Marker.h>
#include <ctime>
ros::Publisher pub_marker;
visualization_msgs::Marker marker;
int ts; // time stamp for start time
#endif
// Subscribed topics string
const std::string IMAGE_STR = "/camera/rgb/image_raw";
const std::string CLOUD_STR = "/camera/depth_registered/points";
const std::string CV_WINDOW_NAME = "Test";
const int EPSILON = 7; // Should be odd number

// Click x and y coordinate in image plane
int click_pixel_x, click_pixel_y;
// Mouse event callback
void mouse_cb(int Event, int x, int y, int flags, void* param)
{
  if(Event == CV_EVENT_LBUTTONUP) {
    click_pixel_x = x;
    click_pixel_y = y;
  }
  //std::cout << "Click on pixel: (" << x << ", " << y << ")" << std::endl;
}
// Subscriber callback:
//  Input: image (of topic: /camera/rgb/image_raw)
//         pointcloud (of topic: /camera/depth_registered/points)
void cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& in)
{
  bool valid_res = true;
  if(DEBUG) ts = clock();
  // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZ> msg_;
  pcl::fromROSMsg(*in, msg_);
  // Convert Image to cv image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  // Show image
  cv::imshow(CV_WINDOW_NAME, cv_ptr->image);
  // If clicked...
  if(click_pixel_x != -1 and click_pixel_y != -1){
    // Get clicked pixel coordinate, use at function
    double click_x = msg_.at(click_pixel_x, click_pixel_y).x,
           click_y = msg_.at(click_pixel_x, click_pixel_y).y,
           click_z = msg_.at(click_pixel_x, click_pixel_y).z;
    if(std::isnan(click_x)){
      // Handle NaN data
      std::cerr << "\033[1;31mGet NaN from raw data, handling...\033[0m" << std::endl; // Red text
      int count = 0;
      double cali_x = 0, cali_y = 0, cali_z = 0;
      for(int i=click_pixel_x-EPSILON/2; i<=click_pixel_x+EPSILON/2; ++i){
        for(int j=click_pixel_y-EPSILON/2; j<=click_pixel_y+EPSILON/2; ++j){
          if(!std::isnan(msg_.at(i, j).x)){
            ++count;
            cali_x += msg_.at(i, j).x;
            cali_y += msg_.at(i, j).y;
            cali_z += msg_.at(i, j).z;
          }
        }
      }
      if(count!=0){
        click_x = cali_x / (float)count;
        click_y = cali_y / (float)count;
        click_z = cali_z / (float)count;
      }
      else {
        valid_res = false;
        std::cerr << "\033[1;31mNeighborhood with 1-norm distance " << EPSILON/2 << " still get NaN \033[0m" << std::endl; // Red text
      }
    }
    if(DEBUG) std::cout << "Execution time: " << (clock()-ts)/double(CLOCKS_PER_SEC)*1000 << " ms" << std::endl;
    double click_x_tf, click_y_tf, click_z_tf;
    if(valid_res){ // If the result is valid then print the information
      std::cout << "Click on pixel: (" << click_pixel_x << ", " << click_pixel_y << ")" << std::endl;
      std::cout << "Coordinate w.r.t. [" << in->header.frame_id << "]: (" <<
                   click_x << " " << click_y << " " << click_z << " )" << std::endl;
      static tf::TransformListener listener;
      tf::StampedTransform stf;
      try {
        listener.waitForTransform("base_link", in->header.frame_id, ros::Time(0), 
                                  ros::Duration(3.0));
        listener.lookupTransform("base_link", in->header.frame_id, ros::Time(0), stf);
        tf::Matrix3x3 rot_mat = tf::Matrix3x3(stf.getRotation());
        tf::Vector3 trans = tf::Vector3(stf.getOrigin()),
                      vec = tf::Vector3(click_x, click_y, click_z),
                  vec_rot = rot_mat*vec,
                   vec_tf = vec_rot + trans;
          click_x_tf = vec_tf.x(); click_y_tf = vec_tf.y(); click_z_tf = vec_tf.z();
      }
      catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        click_x_tf = click_y_tf = click_z_tf = 0;
      }
    }
    std::cout << "Coordinate w.r.t. [base_link]: (" << click_x_tf << ", " 
                                                    << click_y_tf << ", " 
                                                    << click_z_tf << ")" << std::endl;
    if(DEBUG){
      marker.pose.position.x = click_x; 
      marker.pose.position.y = click_y; 
      marker.pose.position.z = click_z;
      pub_marker.publish(marker);
    }
    // Recover pixel coordinates to 0
    click_pixel_x = -1; click_pixel_y = -1;
    cv::waitKey(1.0);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_click_position");
  ros::NodeHandle nh("~");
  cv::namedWindow(CV_WINDOW_NAME);
  cv::setMouseCallback(CV_WINDOW_NAME, mouse_cb, NULL);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, IMAGE_STR, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, CLOUD_STR, 1);
  typedef message_filters::sync_policies::ApproximateTime\
                                   <sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&cb, _1, _2));
  if(DEBUG){
    marker.header.frame_id = "camera_rgb_optical_frame";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01; marker.scale.y = 0.01; marker.scale.z = 0.01;
    marker.color.r = 1.0; marker.color.a = 1.0;
    pub_marker = nh.advertise<visualization_msgs::Marker>("visualize_point", 1);
  }
  ros::spin();
  return 0;
}

