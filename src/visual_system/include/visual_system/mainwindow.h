#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// STL
#include <vector> // std::vector
#include <unordered_map> // std::unordered_map
#include <algorithm> // std::random_shuffle
// QT
#include <QMainWindow>
#include <QTimer>
#include <QThread>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <arm_operation/agent_abb_action.h>
#include <yaml-cpp/yaml.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle pnh, QWidget *parent = 0);
    ~MainWindow();

private:
    bool has_target;
    int tool_id; // Selected tool ID
    int rows, cols;
    const int SAMPLE_CIRCLE_NUM = 100;
    double fx, fy, cx, cy;
    std::vector<int> angle_candidate;
    std::string object_name, color_topic, depth_topic, data_path;
    std::vector<std::string> service_proxies;
    std::unordered_map<std::string, int[3]> map;
    tf::Transform cam2arm;
    geometry_msgs::Point target_pose;
    cv::Mat color_bg, depth_bg;
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient agent_action_server;
    ros::ServiceClient go_home_server;
    ros::ServiceClient vacuum_control_server;
    ros::ServiceClient suction_check_server;
    ros::Publisher pub_marker;
    Ui::MainWindow *ui;
    QTimer *ros_timer;
    void toMarkerMSG(tf::Vector3 target, visualization_msgs::Marker &msg);
    bool getTransform(tf::Transform &t);
    void drawCircle(cv::Mat &src, cv::Point center, int radius=15, int thickness=3);
    bool getIntrinsic(void);
    void get_mask_(void);
    void return_(void);

public slots:
    void spin_timer(void) {ros::spinOnce();}
    void get_mask_cb(void); // `Get Mask` button
    void execute_cb(void); // `Execute` button
    void object_cb(int); // `object_list` combo box
    void tool_cb(int); // `tool_list` combo box
    void success_cb(bool); // `Success` button
    void fail_cb(bool); // `Fail` button
};

#endif // MAINWINDOW_H
