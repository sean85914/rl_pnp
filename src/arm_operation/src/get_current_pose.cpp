#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

// Save the joint angles of specific waypoints to a given file
// Input : file name
// Output: file with six joint angles
// Press 's' to save the waypoint
// and 'e' to exit the process

// Last modify: 09/11
// Editor: Sean
// TODO: make sure there are six joint position

sensor_msgs::JointState js;

void cb_js(const sensor_msgs::JointState msg)
{
    js = msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_current_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, cb_js);

    std::string file_name, file_str;
    ros::param::get("~file_name", file_name);
    std::string path = ros::package::getPath("arm_operation");
    file_str = path + "/data/" + file_name + ".txt";
    std::fstream fp;
    fp.open(file_str.c_str(), std::fstream::out);
    if(!fp){
        ROS_ERROR("Can't open file.");
        return 0;
    }
    char to_do;
    while(1){
        std::cout << "Press 's' to save the waypoint, 'e' to exit the process:" << std::endl;
        std::cin >> to_do;
        if(to_do == 'e'){
            ROS_INFO("End process.");
            break;
        }
        else if(to_do == 's'){
            ros::spinOnce(); // Update data
            for(int i=0;i<6;++i){
                fp << js.position[i] << " ";
            }
            fp << std::endl;
                ROS_INFO("Waypoint saved.");
        }
        else
            ROS_ERROR("No such input.");
    }
    fp.close();
    return 0;
}
