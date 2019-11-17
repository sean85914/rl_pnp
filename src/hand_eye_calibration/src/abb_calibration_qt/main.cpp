#include "mainwindow_calibration.h"
#include <QtCore>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "hand_eye_calibration_abb");
    ros::NodeHandle nh, pnh("~");
    MainWindow w(nh, pnh);
    return a.exec();
}
