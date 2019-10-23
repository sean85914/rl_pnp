// STD
#include <cassert>
#include <map>
// ROS
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
// CV
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>

class CharucoDetector{
 private:
  bool ready;
  bool has_intrinsic;
  int row_;
  int col_;
  int num_; // number of markers
  int bits_; // number of bits in markers
  double square_length_;
  double tag_length_;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> board;
  cv::Mat image_;
  cv::Mat cameraMatrix;
 public:
  CharucoDetector() = default;
  CharucoDetector(int row, int col, int num, int bits, 
                  double square_length, double tag_length);
  void setImage(cv::Mat image) {image_ = image; ready = true;}
  void setIntrinsic(double fx, double fy, double cx, double cy);
  std::map<int, geometry_msgs::Point> getCornersPosition(cv::Mat &drawn, cv::Vec3d &rvec, cv::Vec3d &tvec);
};
