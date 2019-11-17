#ifndef MAINWINDOW_CALIBRATION_H
#define MAINWINDOW_CALIBRATION_H

// STD
#include <utility>

// QT
#include <QMainWindow>
#include <QTimer>
#include <QThread>

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <abb_node/robot_GetCartesian.h>

// OpenCV
#include <opencv2/core.hpp>

// Charuco detection wrapper
#include <visual_system/charuco_detector.h>

// Point set registration wrapper
#include <point_set_registration.h>

const int MAX_DATA_POINTS_REQUIRED = 3;
typedef std::map<int, geometry_msgs::Point> CornerMap;
typedef std::pair<geometry_msgs::Point, geometry_msgs::Point> DataPair;
typedef std::vector<DataPair> DataPairArray;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle pnh,QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    bool has_result, first;
    std::string camera_frame;
    const std::string get_robot_pose_server;
    QTimer *broadcast_timer, *ros_timer;
    ros::NodeHandle nh_, pnh_;
    std::vector<double> charuco_offset;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    typedef message_filters::sync_policies::ApproximateTime\
                                          <sensor_msgs::Image, 
                                           sensor_msgs::CameraInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    CharucoDetector detector;
    CornerMap id_corner_map;
    DataPairArray data_pair_array;
    tf::Transform result_transform;
    void callback(const sensor_msgs::ImageConstPtr&,
                  const sensor_msgs::CameraInfoConstPtr&);

public slots:
    void spin_timer(void){
      ros::spinOnce();
    }
    void broadcast_callback(void){
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(result_transform, ros::Time::now(), "base_link", camera_frame));
    }
    void exit_program(void);
    void record_data(void);
    void compute_result(void);
    void broadcast_transform(void);
};


#endif // MAINWINDOW_CALIBRATION_H
