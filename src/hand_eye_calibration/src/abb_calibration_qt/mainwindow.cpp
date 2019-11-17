#include "mainwindow_calibration.h"
#include "ui_mainwindow_calibration.h"

const int ROW = 6;
const int COL = 3;
const int NUM = 4;
const int BITS = 50;
const double SQUARE_LEN = 0.04f;
const double TAG_LEN = 0.032f;

// Conversion
std::string tf2stringInfo(const tf::Transform t){
    std::string info;
    tf::Vector3 trans = t.getOrigin();
    tf::Quaternion quat = t.getRotation();
    tf::Matrix3x3 mat = t.getBasis();
    double r, p, y;
    mat.getRPY(r, p, y);
    info = "Translation: " + std::to_string(trans.getX()) 
                    + ", " + std::to_string(trans.getY()) 
                    + ", " + std::to_string(trans.getZ()) + "\n";
    info += "Orientation(Quaternion): " + std::to_string(quat.getX()) 
                                 + ", " + std::to_string(quat.getY())
                                 + ", " + std::to_string(quat.getZ())
                                 + ", " + std::to_string(quat.getW()) + "\n";
    info += "Orientation(RPY): " + std::to_string(r)
                          + ", " + std::to_string(p)
                          + ", " + std::to_string(y) + "\n";
    return info;
}

MainWindow::MainWindow(ros::NodeHandle nh, ros::NodeHandle pnh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    nh_(nh),
    pnh_(pnh),
    has_result(false),
    first(true),
    get_robot_pose_server("/abb/robot_GetCartesian")
{
    ui->setupUi(this);
    detector = CharucoDetector(ROW, COL, NUM, BITS, SQUARE_LEN, TAG_LEN);
    charuco_offset.resize(3);
    if(!pnh_.getParam("charuco_offset", charuco_offset)){
        ROS_ERROR("No charuco offset data provided, force terminated...");
        exit(EXIT_FAILURE);
    }
    connect(ui->record_button, SIGNAL(clicked()), SLOT(record_data(void)));
    connect(ui->compute_button, SIGNAL(clicked()), SLOT(compute_result(void)));
    connect(ui->broadcast_button, SIGNAL(clicked()), SLOT(broadcast_transform(void)));
    connect(ui->exit_button, SIGNAL(clicked()), SLOT(exit_program(void)));
    image_sub.subscribe(nh_, "image", 1);
    info_sub.subscribe(nh_, "camera_info", 1);
    sync.reset(new Sync(MySyncPolicy(10), image_sub, info_sub));
    sync->registerCallback(boost::bind(&MainWindow::callback, this, _1, _2));
    ros_timer = new QTimer(this);
    broadcast_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spin_timer()));
    ui->textBrowser->setText("Welcom to hand-eye calibration process,\n\
press `Record` to record data, \n\
`Compute` to get the transformation result, \n\
`Broadcast` to broadcast the computed transform and \n\
`Exit` to turn off the program. \n\
You have " + QString::number(data_pair_array.size()) + " data now.");
    ros_timer->start(33); // 30 Hz
    show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::record_data(void){
    // Call service to get robot arm end effector pose
    ros::ServiceClient client = nh_.serviceClient<abb_node::robot_GetCartesian>(get_robot_pose_server);
    if(id_corner_map.size()==0){
        ui->textBrowser->setText("You have " + QString::number(data_pair_array.size()) + \
                                 " data now. \nNo corner detected, abort request...");
        return;
    }
    abb_node::robot_GetCartesian srv;
    if(!client.call(srv)){
        ui->textBrowser->setText("You have " + QString::number(data_pair_array.size()) + \
                                 " data now. \nCan't get robot pose, abort request...");
        return;
    }
    tf::Vector3 ee_position(srv.response.x/100.0, srv.response.y/100.0, srv.response.z/100.0);
    tf::Quaternion ee_orientation(srv.response.qx, srv.response.qy, srv.response.qz, srv.response.q0);
    tf::Transform ee_transform(ee_orientation, ee_position);
    for(auto x: id_corner_map){
      int id = x.first;
      geometry_msgs::Point corner_point_eye = x.second;
      int z_offset_unit = id%(ROW-1)+1;
      int y_offset_unit = id/(ROW-1)+1;
      tf::Vector3 corner_offset(charuco_offset[0], \
                                charuco_offset[1]+y_offset_unit*SQUARE_LEN, \
                                charuco_offset[2]+z_offset_unit*SQUARE_LEN);
      corner_offset = ee_transform*corner_offset;
      geometry_msgs::Point corner_point_hand;
      corner_point_hand.x = corner_offset.getX();
      corner_point_hand.y = corner_offset.getY();
      corner_point_hand.z = corner_offset.getZ();
      data_pair_array.push_back(std::pair<geometry_msgs::Point, geometry_msgs::Point>(corner_point_hand, corner_point_eye));
    }
    ui->textBrowser->setText("Got " + QString::number(id_corner_map.size()) + \
                             " data\nYou have " + QString::number(data_pair_array.size()) + " data now.");
}

void MainWindow::compute_result(void){
    if(data_pair_array.size()<MAX_DATA_POINTS_REQUIRED)
        ui->textBrowser->setText("You have " + QString::number(data_pair_array.size()) + " data now. \n\
You don't have enough data points, abord request...");
    else{
        PointSetRegistration psr(data_pair_array);
        double mm_error;
        psr.getData(mm_error, result_transform);
        ui->textBrowser->setText("Compute result:\n"+QString::fromUtf8(tf2stringInfo(result_transform).c_str())+"Compute error: " + QString::number(mm_error) + " (mm)\n");
        has_result = true;
    }
}

void MainWindow::broadcast_transform(void){
    static bool is_broadcasting = false;
    if(!has_result) ui->textBrowser->setText("You have " + QString::number(data_pair_array.size()) + " data now. \n\
You haven't compute the result, abord request...");
    else{
        if(!is_broadcasting){
            // Start a thread to broadcast transform, and disable record and compute button
            is_broadcasting = true;
            ui->record_button->setEnabled(false);
            ui->compute_button->setEnabled(false);
            result_transform = tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)); // dummy one for test
            ui->textBrowser->setText("Broadcasting transform, press `Broadcast` to stop\n"+\
                                     QString::fromUtf8(tf2stringInfo(result_transform).c_str()));
            connect(broadcast_timer, SIGNAL(timeout()), this, SLOT(broadcast_callback()));
            broadcast_timer->start(10);
        }else{ 
            // Stop broadcast thread, and enable record and compute button
            broadcast_timer->stop();
            is_broadcasting = false;
            ui->record_button->setEnabled(true);
            ui->compute_button->setEnabled(true);
            ui->textBrowser->setText("Stop broadcast");
        } 
    }
}

void MainWindow::exit_program(void){
    MainWindow::close();
    ros_timer->stop();
}

// ROS

void MainWindow::callback(const sensor_msgs::ImageConstPtr      &image, 
                          const sensor_msgs::CameraInfoConstPtr &info){
    camera_frame = info->header.frame_id;
    double fx = info->K[0],
           fy = info->K[4],
           cx = info->K[2],
           cy = info->K[5];
    detector.setIntrinsic(fx, fy, cx, cy);
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    detector.setImage(cv_ptr->image);
    cv::Vec3d rvec, tvec;
    cv::Mat draw_img;
    id_corner_map = detector.getCornersPosition(draw_img, rvec, tvec);
    cv::cvtColor(draw_img, draw_img, CV_BGR2RGB);
    ui->detection_view->setPixmap(QPixmap::fromImage(QImage(draw_img.data, \
                                                            draw_img.cols, \
                                                            draw_img.rows, \
                                                            draw_img.step, 
                                                            QImage::Format_RGB888)));
}
