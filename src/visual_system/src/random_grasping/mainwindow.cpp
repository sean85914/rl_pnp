#include "visual_system/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle nh, ros::NodeHandle pnh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    nh_(nh), pnh_(pnh), 
    tool_id(1), 
    has_target(false),
    angle_candidate(std::vector<int>(4, 0)),
    color_topic("/camera1/color/image_raw"),
    depth_topic("/camera1/aligned_depth_to_color/image_raw")
{
    this->setFixedSize(QSize(1280, 640));
    for(int i=0; i<4; ++i)
        angle_candidate[i] = -90+45*i;
    // Load images
    data_path = ros::package::getPath("visual_system")+"/data/";
    color_bg = cv::imread(data_path+"color_background.jpg", CV_LOAD_IMAGE_COLOR);
    depth_bg = cv::imread(data_path+"depth_background.png", CV_LOAD_IMAGE_ANYDEPTH);
    rows = color_bg.rows; cols = color_bg.cols;
    // Cast data
    color_bg.convertTo(color_bg, CV_64FC3, 1./256);
    depth_bg.convertTo(depth_bg, CV_64FC1, 1./1000);
    // ROS
    pub_marker = pnh_.advertise<visualization_msgs::Marker>("marker", 1);
    service_proxies.resize(4);
    service_proxies[0] = "/agent_server_node/agent_take_action";
    service_proxies[1] = "/agent_server_node/go_home";
    service_proxies[2] = "/vacuum_pump_control_node/vacuum_control";
    service_proxies[3] = "/vacuum_pump_control_node/check_suck_success";
    for(auto s: service_proxies){
      while(ros::ok()&&!ros::service::waitForService(s, ros::Duration(3.0))){}
    }
    agent_action_server = pnh_.serviceClient<arm_operation::agent_abb_action>(service_proxies[0]);
    go_home_server = pnh_.serviceClient<std_srvs::Empty>(service_proxies[1]);
    vacuum_control_server = pnh_.serviceClient<std_srvs::SetBool>(service_proxies[2]);
    suction_check_server = pnh_.serviceClient<std_srvs::SetBool>(service_proxies[3]);
    bool not_ready = (!agent_action_server.exists()||!go_home_server.exists()||!vacuum_control_server.exists()||!suction_check_server.exists());
    if(not_ready){
      qFatal("\033[1;33mServices not ready yet\033[0m");
      MainWindow::close();
      return;
    }
    // Initialize widget
    ui->setupUi(this);
    ui->execute_button->setEnabled(false);
    ui->success_button->setEnabled(false);
    ui->fail_button->setEnabled(false);
    ui->status_browser->setText("Ready");
    // Timer
    ros_timer = new QTimer(this);
    // Signal & Slot
    connect(ui->get_mask_button, SIGNAL(clicked()), SLOT(get_mask_cb(void)));
    connect(ui->object_list, SIGNAL(currentIndexChanged(int)), SLOT(object_cb(int)));
    connect(ui->execute_button, SIGNAL(clicked()), SLOT(execute_cb(void)));
    connect(ui->tool_list, SIGNAL(currentIndexChanged(int)), SLOT(tool_cb(int)));
    connect(ui->success_button, SIGNAL(clicked(bool)), SLOT(success_cb(bool)));
    connect(ui->fail_button, SIGNAL(clicked(bool)), SLOT(fail_cb(bool)));
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spin_timer()));
    ros_timer->start(33); // 30 Hz
    if(!getIntrinsic()||!getTransform(cam2arm)){
      qFatal("\033[1;33mCan't get necessary data\033[0m");
      MainWindow::close();
      return;
    }
    // Initialize the robot
    std_srvs::Empty empty_srv;
    std_srvs::SetBool setBool_srv;
    setBool_srv.request.data = false;
    go_home_server.call(empty_srv);
    vacuum_control_server.call(setBool_srv);
    show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::toMarkerMSG(tf::Vector3 target, visualization_msgs::Marker &msg){
    msg.header.frame_id = "base_link";
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.scale.x = msg.scale.y = msg.scale.z = 0.02f;
    msg.color.r = msg.color.a = 1.0f;
    msg.pose.position.x = target.getX();
    msg.pose.position.y = target.getY();
    msg.pose.position.z = target.getZ();
    msg.pose.orientation.w = 1.0f;
}

bool MainWindow::getTransform(tf::Transform &t){
    tf::TransformListener listener;
    tf::StampedTransform stf;
    try{
        listener.waitForTransform("base_link", "camera1_color_optical_frame", ros::Time(0), ros::Duration(0.5));
        listener.lookupTransform("base_link", "camera1_color_optical_frame", ros::Time(0), stf);
        t = tf::Transform(stf.getRotation(), stf.getOrigin());
        return true;
    }catch(tf::TransformException &e){
        ROS_WARN("%s", e.what());
        return false;
    }
}

bool MainWindow::getIntrinsic(void){
    sensor_msgs::CameraInfoConstPtr camera_info;
    // Replace `image_raw` to `camera_info`
    std::string camera_info_topic(color_topic);
    camera_info_topic.replace(color_topic.length()-9, 9, "camera_info");
    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(0.5));
    if(!camera_info){
        ROS_ERROR("Can't get camera info");
        return false;
    }
    fx = camera_info->K[0];
    fy = camera_info->K[4];
    cx = camera_info->K[2];
    cy = camera_info->K[5];
    return true;
}

void MainWindow::drawCircle(cv::Mat &src, cv::Point center, int radius, int thickness){
    int cnt = 0;
    for(int i=0; i<=thickness; ++i){
        for(int j=0; j<SAMPLE_CIRCLE_NUM; ++j){
            ++cnt;
            double theta = 2*M_PI/SAMPLE_CIRCLE_NUM*j;
            cv::Point p(center.x+(radius+i)*cos(theta), center.y+(radius+i)*sin(theta));
            src.at<unsigned char>(p) = 125;
        }
    }
}

void MainWindow::get_mask_(void){
    sensor_msgs::ImageConstPtr color_frame, depth_frame;
    color_frame = ros::topic::waitForMessage<sensor_msgs::Image>(color_topic, ros::Duration(0.5));
    depth_frame = ros::topic::waitForMessage<sensor_msgs::Image>(depth_topic, ros::Duration(0.5));
    if(!color_frame||!depth_frame){
        ROS_ERROR("Can't get images");
        MainWindow::close();
    }
    cv_bridge::CvImagePtr color_ptr, depth_ptr;
    try{
        color_ptr = cv_bridge::toCvCopy(*color_frame, sensor_msgs::image_encodings::BGR8);
        depth_ptr = cv_bridge::toCvCopy(*depth_frame, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("%s", e.what());
        MainWindow::close();
    }
    cv::Mat current_color, current_depth;
    color_ptr->image.convertTo(current_color, CV_64FC3, 1./256);
    depth_ptr->image.convertTo(current_depth, CV_64FC1, 1./1000);
    // ------------------------------------------------------------------------------------------
    cv::Mat color_diff, depth_diff;
    cv::absdiff(current_color, color_bg, color_diff);
    cv::absdiff(current_depth, depth_bg, depth_diff);
    // Declare foreground mask placeholder
    cv::Mat fg_color, fg_depth, fg_mask;
    fg_color = cv::Mat::zeros(rows, cols, CV_8UC1);
    fg_depth = cv::Mat::zeros(rows, cols, CV_8UC1);
    fg_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    for(int i=0; i<rows; ++i){
      for(int j=0; j<cols; ++j){
        cv::Vec3d p = color_diff.at<cv::Vec3d>(i, j);
        if(p[0]>=0.3||p[1]>=0.3||p[2]>=0.3)
          fg_color.at<unsigned char>(i, j) = 255;
        if(depth_diff.at<double>(i, j)>=0.02&&depth_bg.at<double>(i, j)!=0&&current_depth.at<double>(i, j)!=0)
          fg_depth.at<unsigned char>(i, j) = 255;
      }
    }
    cv::bitwise_and(fg_color, fg_depth, fg_mask);
    std::vector<cv::Point> non_zeros;
    cv::findNonZero(fg_mask, non_zeros);
    std::random_shuffle(non_zeros.begin(), non_zeros.end());
    cv::Point selected = *non_zeros.begin();
    drawCircle(fg_mask, selected);
    double cam_x, cam_y, cam_z = current_depth.at<double>(selected);
    cam_x = (selected.x-cx)/fx*cam_z;
    cam_y = (selected.y-cy)/fy*cam_z;
    tf::Vector3 target(cam_x, cam_y, cam_z), target_arm = cam2arm*target;
    QString info = QString::number(selected.x) + " " + QString::number(selected.y);
    info += ("| -> " + QString::number(target_arm.getX()) + " " \
            + QString::number(target_arm.getY()) + " "\
            + QString::number(target_arm.getZ()));
    ui->status_browser->setText(info);
    //ROS_INFO("%d %d| %f %f %f -> %f %f %f", selected.x, selected.y, cam_x, cam_y, cam_z, target_arm.getX(), target_arm.getY(), target_arm.getZ());
    visualization_msgs::Marker msg;
    toMarkerMSG(target_arm, msg);
    target_pose.x = target_arm.getX();
    target_pose.y = target_arm.getY();
    target_pose.z = target_arm.getZ();
    pub_marker.publish(msg);
    #ifdef SAVE_IMAGE
    cv::imwrite(data_path+"mask_color.jpg", fg_color);
    cv::imwrite(data_path+"mask_depth.jpg", fg_depth);
    cv::imwrite(data_path+"mask_res.jpg",fg_mask);
    cv::imwrite(data_path+"current_color.jpg", color_ptr->image);
    #endif
    cv::Mat resized;
    cv::resize(fg_mask, resized, cv::Size(ui->image_view->width(), ui->image_view->height()));
    ui->image_view->setPixmap(QPixmap::fromImage(QImage(resized.data, \
                                                        resized.cols, \
                                                        resized.rows, \
                                                        resized.step, 
                                                        QImage::Format_Grayscale8)));
}

void MainWindow::get_mask_cb(void){
    get_mask_();
    has_target = true;
    ui->execute_button->setEnabled(true);
}

void MainWindow::execute_cb(void){
    ui->execute_button->setEnabled(false);
    double theta = 0.0;
    if(tool_id==1){
        std::random_shuffle(angle_candidate.begin(), angle_candidate.end());
        theta = *angle_candidate.begin()/360.0*M_PI;
    }
    QString info = "Target: " + QString::number(target_pose.x) + " " \
                              + QString::number(target_pose.y) + " " \
                              + QString::number(target_pose.z) + "\n";
    info += "Tool: " + QString::number(tool_id) + "\n";
    info += "Angle: " + QString::number(theta) + "\n";
    ui->status_browser->setText(info);
    QApplication::processEvents();
    arm_operation::agent_abb_action action;
    action.request.tool_id = tool_id;
    action.request.position = target_pose;
    action.request.angle = theta;
    agent_action_server.call(action);
    std_srvs::Empty empty_srv;
    go_home_server.call(empty_srv);
    has_target = false;
}

void MainWindow::object_cb(int num){
    ui->status_browser->setText(ui->object_list->currentText());
}

void MainWindow::tool_cb(int num){
    if(ui->tool_list->currentText()=="I")
        tool_id = 1;
    else if(ui->tool_list->currentText()=="II")
        tool_id = 2;
    else
        tool_id = 3;
    ui->status_browser->setText(ui->tool_list->currentText());
}

void MainWindow::success_cb(bool data){
    // TODO
    return_();
}

void MainWindow::fail_cb(bool data){
    // TODO
    return_();
}

void MainWindow::return_(void){

}
