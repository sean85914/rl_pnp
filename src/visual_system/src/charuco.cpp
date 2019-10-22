#include <sstream>
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>

const int ROW = 6;
const int COL = 3;
const double SQUARE_LEN = 0.04f;
const double TAG_LEN = 0.032f;
cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Ptr<cv::aruco::CharucoBoard> board;
ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr &image, 
              const sensor_msgs::CameraInfoConstPtr &info){
  ros::Time ts = ros::Time::now();
  double fx = info->K[0];
  double fy = info->K[4];
  double cx = info->K[2];
  double cy = info->K[5];
  cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat draw_img;
  try{
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_ptr->image.copyTo(draw_img);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
  int original_markers = ids.size();
  cv::aruco::refineDetectedMarkers(cv_ptr->image, 
                                   board, 
                                   corners, 
                                   ids, 
                                   cv::noArray(),
                                   cameraMatrix);
  if(ids.size()==0){ROS_WARN("No marker detected, abort..."); return;}
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  int corners_amount = cv::aruco::interpolateCornersCharuco(corners, 
                                                            ids, 
                                                            cv_ptr->image, 
                                                            board,
                                                            charucoCorners,
                                                            charucoIds,
                                                            cameraMatrix);
  cv::Vec3d rvec, tvec;
  bool valid_pose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, 
                                                        charucoIds,
                                                        board,
                                                        cameraMatrix,
                                                        cv::noArray(),
                                                        rvec, tvec);
  if(!valid_pose) {ROS_WARN("Invalid pose estimation, abort..."); return;}
  cv::aruco::drawDetectedCornersCharuco(draw_img, 
                                        charucoCorners, 
                                        charucoIds, 
                                        cv::Scalar(255, 0, 0));
  cv::aruco::drawAxis(draw_img, cameraMatrix, cv::noArray(), rvec, tvec, 0.1);
  sensor_msgs::Image pub_image;
  cv_bridge::CvImage out_msg;
  out_msg.header.frame_id = image->header.frame_id;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = draw_img;
  pub.publish(out_msg);
  // Convert rotation vector to matrix
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
  tf::Matrix3x3 tf_rot_matrix(rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2), 
                              rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2), 
                              rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));
  tf::Vector3 origin(tvec[0], tvec[1], tvec[2]);
  // Broadcast transformation
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setBasis(tf_rot_matrix); transform.setOrigin(origin);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), image->header.frame_id, "charuco"));
  for(int i=0; i<charucoIds.size(); ++i){
    int x_offset_unit = charucoIds[i]%(ROW-1)+1;
    int y_offset_unit = charucoIds[i]/(ROW-1)+1;
    tf::Vector3 corner_origin = tf_rot_matrix*tf::Vector3(x_offset_unit*SQUARE_LEN, y_offset_unit*SQUARE_LEN, 0.0f);
    corner_origin += origin;
    tf::Transform corner_transform(transform);
    corner_transform.setOrigin(corner_origin);
    br.sendTransform(tf::StampedTransform(corner_transform, ros::Time::now(), image->header.frame_id, "tag_"+std::to_string(charucoIds[i])));
  }
  ros::Duration(1.0f/30.0f).sleep();
  //ROS_INFO("Process time: %f", (ros::Time::now()-ts).toSec());
  return;
}

int main(int argc, char** argv)
{
  dictionary.reset(new cv::aruco::Dictionary());
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  board.reset(new cv::aruco::CharucoBoard());
  board = cv::aruco::CharucoBoard::create(ROW, COL, SQUARE_LEN, TAG_LEN, dictionary);
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
