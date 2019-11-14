// STD
#include <cassert>
#include <map>
// ROS
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
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
  bool ready; // Is image set?
  bool has_intrinsic; // Is the detector has intrinsic matrix?
  // Charuco information data, should be provided in constructor
  int row_; // Row of charuco
  int col_; // Column of charuco
  int num_; // number of markers
  int bits_; // number of bits in markers
  double square_length_; // Square length
  double tag_length_; // Tag length
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> board;
  cv::Mat image_;  // Target image to find marker
  cv::Mat cameraMatrix; // Intrinsic matrix, we assumed user using image from Intel Realsense depth camera, so the distortion matrix set to zero matrix
 public:
  CharucoDetector() = default;
  CharucoDetector(int row, int col, int num, int bits, 
                  double square_length, double tag_length);
  void setImage(cv::Mat image) {image_ = image; ready = true;}
  void setIntrinsic(double fx, double fy, double cx, double cy);
  /*
   * Get corners position and its corresponding ID
   * [out] cv::Mat &drawn: image with drawn information, can visualized detection result
   * [out] cv::Vec3d &rvec: charuco board orientation w.r.t. image frame, can further change to matrix format by `cv::Rodrigues`
   * [out] cv::Vec3d &tvec: charuco board position w.r.t. image frame, notice that it represent the left bottom corner of the board
   * [out] std::map<int, geometry_msgs::Point>: map, which bind corner ID and its position in camera frame
   */
  std::map<int, geometry_msgs::Point> getCornersPosition(cv::Mat &drawn, cv::Vec3d &rvec, cv::Vec3d &tvec);
};
