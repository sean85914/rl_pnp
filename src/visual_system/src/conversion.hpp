#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

tf::Matrix3x3 rvec2tf(const cv::Vec3d rvec){
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
  tf::Matrix3x3 tf_rot_matrix(
rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2), 
rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2), 
rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));
  return tf_rot_matrix;
}

tf::Vector3 tvec2tf(const cv::Vec3f tvec){
  tf::Vector3 origin(tvec[0], tvec[1], tvec[2]);
  return origin;
}

tf::Vector3 point2tf(const geometry_msgs::Point p){
  tf::Vector3 origin(p.x, p.y, p.z);
  return origin;
}
