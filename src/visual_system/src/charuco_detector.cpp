#include <visual_system/charuco_detector.h>

CharucoDetector::CharucoDetector(int row, int col, int num, int bits, 
                                 double square_length, double tag_length):
  ready(false), has_intrinsic(false),
  row_(row), col_(col), num_(num), bits_(bits), 
  square_length_(square_length), tag_length_(tag_length){
  // See `opencv2/aruco/dictionary.hpp`
  assert((num_==4 or num_==5 or num_==6 or num_==7));
  assert((bits_==50 or bits_==100 or bits_==250 or bits_==1000));
  dictionary.reset(new cv::aruco::Dictionary());
  board.reset(new cv::aruco::CharucoBoard());
  int dictionary_num = (num_-4)*4;
  switch(bits_){
    case(50):
      dictionary_num += 0;
      break;
    case(100):
      dictionary_num += 1;
      break;
    case(250):
      dictionary_num += 2;
      break;
    case(1000):
      dictionary_num += 3;
      break;
  }
  dictionary = cv::aruco::getPredefinedDictionary(dictionary_num);
  board = cv::aruco::CharucoBoard::create(row_, col_, square_length_, tag_length_, dictionary);
}

void CharucoDetector::setIntrinsic(double fx, double fy, double cx, double cy){
  cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  has_intrinsic = true;
}

std::map<int, geometry_msgs::Point> CharucoDetector::getCornersPosition(cv::Mat &drawn, cv::Vec3d &rvec, cv::Vec3d &tvec){
  // Refer: https://docs.opencv.org/3.4/df/d4a/tutorial_charuco_detection.html
  std::map<int, geometry_msgs::Point> point_map;
  assert(ready and has_intrinsic);
  image_.copyTo(drawn);
  // Detect markers in the image
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image_, dictionary, corners, ids);
  cv::aruco::refineDetectedMarkers(image_, board, corners, ids, cv::noArray(), cameraMatrix);
  if(ids.size()==0) return point_map; // Return empty map if no corner detected
  // Detect corners in the image
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  int corners_amount = cv::aruco::interpolateCornersCharuco(corners,
                                                            ids,
                                                            image_,
                                                            board,
                                                            charucoCorners,
                                                            charucoIds,
                                                            cameraMatrix);
  // Estimate pose of the charuco (Left bottom corner as origin, row as X-axis and column as Y-axis)
  bool valid_pose = cv::aruco::estimatePoseCharucoBoard(charucoCorners,
                                                        charucoIds,
                                                        board,
                                                        cameraMatrix,
                                                        cv::noArray(),
                                                        rvec, tvec);
  if(!valid_pose) return point_map; // Not valid pose, return empty map
  // Draw corner and charuco coordinate in image
  cv::aruco::drawDetectedCornersCharuco(drawn, 
                                        charucoCorners, 
                                        charucoIds, 
                                        cv::Scalar(255, 0, 0));
  cv::aruco::drawAxis(drawn, cameraMatrix, cv::noArray(), rvec, tvec, 0.1);
  // Get each corner position in image coordinate (for image from Intel RealSense, it's `camera_color_optical_frame`)
  // Convert rotation vector to matrix
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
  tf::Matrix3x3 tf_rot_matrix(rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2), 
                              rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2), 
                              rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));
  tf::Vector3 origin(tvec[0], tvec[1], tvec[2]);
  for(int i=0; i<charucoIds.size(); ++i){
    int x_offset_unit = charucoIds[i]%(row_-1)+1;
    int y_offset_unit = charucoIds[i]/(row_-1)+1;
    tf::Vector3 corner_origin = tf_rot_matrix*tf::Vector3(x_offset_unit*square_length_, y_offset_unit*square_length_, 0.0f);
    corner_origin += origin;
    geometry_msgs::Point p; 
    p.x = corner_origin.getX();
    p.y = corner_origin.getY();
    p.z = corner_origin.getZ();
    point_map.insert(std::pair<int, geometry_msgs::Point>(charucoIds[i], p));
  }
  ready = false;
  return point_map;
}
