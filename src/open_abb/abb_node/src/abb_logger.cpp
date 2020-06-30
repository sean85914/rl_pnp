#include <cstdio>
#include <string>
#include <cstring>
#include <sstream>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <abb_node/joints.h>
#include <abb_node/robot_GetCartesian.h>

const int MAX_LEN = 1024;

class TCPSocket{
 private:
   int logger_port_, logger_socket;
   char message[MAX_LEN];
   std::string host_;
   struct sockaddr_in socket_info;
   socklen_t addr_size;
 public:
  TCPSocket()= default;
  TCPSocket(std::string host, int logger_port): host_(host), logger_port_(logger_port){
    logger_socket = socket(AF_INET, SOCK_STREAM, 0);
    bzero(&socket_info, sizeof(socket_info));
    socket_info.sin_family = AF_INET;
    socket_info.sin_port = htons(logger_port);
    socket_info.sin_addr.s_addr = inet_addr(host.c_str());
    if(logger_socket==-1){
      printf("\033[0;31mFailed to connet to %s:%d\033[0m\n", host.c_str(), logger_port_);
    }
    int err = connect(logger_socket, (struct sockaddr *)&socket_info, sizeof(socket_info));
    if(err==-1){
      printf("Can't connect to port: %d\n", logger_port_);
    }else{
      printf("Connect to %s:%d\n", host.c_str(), logger_port_);
    }
    recv(logger_socket, message, MAX_LEN, 0);
  }
  std::string getString(void){
    int len = recv(logger_socket, message, MAX_LEN, 0);
    message[len] = '\0';
    return std::string(message);
  }
  ~TCPSocket(){
    close(logger_socket);
  }
};

class ROSWrapper{
 private:
  bool first;
  double x, y, z, q0, qx, qy, qz;
  double last_joint[6];
  ros::Time last_time;
  ros::NodeHandle nh_, pnh_;
  sensor_msgs::JointState js_msg;
  ros::Publisher pub_joints, pub_pose, pub_js;
  ros::ServiceServer get_pose_server;
  bool service_cb(abb_node::robot_GetCartesian::Request  &req, 
                  abb_node::robot_GetCartesian::Response &res){
    res.x = x;
    res.y = y;
    res.z = z;
    res.q0 = q0;
    res.qx = qx;
    res.qy = qy;
    res.qz = qz;
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
 public:
  ROSWrapper(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh_), first(true){
    js_msg.name.resize(6); js_msg.position.resize(6); js_msg.velocity.resize(6);
    for(int i=1; i<=6; ++i){
      js_msg.name[i-1] = "joint" + std::to_string(i);
    }
    pub_joints = nh_.advertise<abb_node::joints>("/abb/joints", 1);
    pub_pose   = nh_.advertise<geometry_msgs::Pose>("/abb/pose", 1);
    pub_js     = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    get_pose_server = nh_.advertiseService("/abb/robot_GetCartesian", &ROSWrapper::service_cb, this);
  }
  void parseData(std::string str){
    // Valid string starts with `#`
    if(str[0]!='#') return;
    // Following with start `#`, it is code which indicates which information
    int code = str[2]-'0';
    int space_counter = 0;
    while(space_counter!=5){
      std::size_t pos = str.find(" ");
      str = str.substr(pos+1);
      ++space_counter;
    }
    switch(code){
      case 0: {// Pose
        // # 0 [data] [time] [timer] [x] [y] [z] [q0] [qx] [qy] [qz]
        std::stringstream ss(str);
        ss >> x >> y >> z >> q0 >> qx >> qy >> qz;
        geometry_msgs::Pose pose;
        pose.position.x = x/1000.0f; pose.position.y = y/1000.0f; pose.position.z = z/1000.0f; 
        pose.orientation.x = qx; pose.orientation.y = qy; pose.orientation.z = qz; pose.orientation.w = q0; 
        pub_pose.publish(pose);
        break;
      }
      case 1: {// Joints
        double js[6];
        std::stringstream ss(str);
        ss >> js[0] >> js[1] >> js[2] >> js[3] >> js[4] >> js[5];
        abb_node::joints joints;
        std::copy(js, js+6, joints.joint.begin());
        std::copy(js, js+6, js_msg.position.begin());
        for(int i=0; i<6; ++i){
          joints.joint[i] *= M_PI/180.0;
          js_msg.position[i] *= M_PI/180.0;
        }
        js_msg.header.stamp = ros::Time::now();
        if(first)
          first = true;
        else{
          for(int i=0; i<6; ++i)
            js_msg.velocity[i] = (js_msg.position[i]-js[i])/(js_msg.header.stamp-last_time).toSec();
        }
        last_time = js_msg.header.stamp;
        std::copy(js, js+6, last_joint);
        pub_js.publish(js_msg);
        pub_joints.publish(joints);
        break;
      }
    }
    ros::spinOnce();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_logger");
  ros::NodeHandle nh, pnh("~");
  TCPSocket foo("192.168.125.1", 5001);
  ROSWrapper boo(nh, pnh);
  while(ros::ok()) boo.parseData(foo.getString());
  
  return 0;
}
