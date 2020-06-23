#include "visual_system/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "random_grasping");
    ros::NodeHandle nh, pnh("~");
    MainWindow w(nh, pnh);
    return a.exec();

}
