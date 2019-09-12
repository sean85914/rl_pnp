#include <ros/ros.h>
#include <serial/serial.h>
// SRV
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <vacuum_conveyor_control/vacuum_control.h>

class ArduinoControl{
 private:
  int baudrate;
  const int TO = 50; // Serial timeout, not sure what this really are
  int vacuum_thres; // Volotage from sensor higher than this value will consider as fail
  std::string port;
  serial::Serial mySerial;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer vacuum_srv;
  ros::ServiceServer conveyor_srv;
  ros::ServiceServer pheumatic_srv;
  ros::ServiceServer check_success_srv;
  bool vacuum_control_cb(vacuum_conveyor_control::vacuum_control::Request&, vacuum_conveyor_control::vacuum_control::Response&);
  bool conveyor_control_cb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool pheumatic_control_cb(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  bool check_suck_success(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
 public:
  ArduinoControl(ros::NodeHandle, ros::NodeHandle);
  ~ArduinoControl(){mySerial.close();}
};

ArduinoControl::ArduinoControl(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
  if(!pnh_.getParam("baudrate", baudrate)) baudrate = 115200; ROS_INFO("baudrate: %d", baudrate);
  if(!pnh_.getParam("port", port)) port = "/dev/ttyACM0"; ROS_INFO("port: %s", port.c_str());
  if(!pnh_.getParam("vacuum_thres", vacuum_thres)) vacuum_thres = 800; ROS_INFO("vacuum_thres: %d", vacuum_thres);
  // Open serial
  mySerial.setPort(port);
  mySerial.setBaudrate(baudrate);
  serial::Timeout to = serial::Timeout::simpleTimeout(TO);
  mySerial.setTimeout(to);
  mySerial.open();
  if(!mySerial.isOpen()){
    ROS_ERROR("Cannot open specific port, exiting...");
    ros::shutdown();
  }
  // Advertise services
  vacuum_srv = pnh_.advertiseService("vacuum_control", &ArduinoControl::vacuum_control_cb, this);
  conveyor_srv = pnh_.advertiseService("conveyor_control", &ArduinoControl::conveyor_control_cb, this);
  pheumatic_srv = pnh_.advertiseService("pheumatic_control", &ArduinoControl::pheumatic_control_cb, this);
  check_success_srv = pnh_.advertiseService("check_suck_success", &ArduinoControl::check_suck_success, this);
}

bool ArduinoControl::vacuum_control_cb(vacuum_conveyor_control::vacuum_control::Request &req, 
                                       vacuum_conveyor_control::vacuum_control::Response &res){
  std::string command = "v" + std::to_string(req.command);
  ROS_INFO("Receive vacuum command: %d", req.command);
  mySerial.write(command);
  return true;
}

bool ArduinoControl::conveyor_control_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  std::string command = "c1";
  ROS_INFO("Receive conveyor command");
  mySerial.write(command);
  return true;
}

bool ArduinoControl::pheumatic_control_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  std::string command;
  if(req.data) {
    command = "p1";
    pnh_.setParam("pheumatic", true);
  }
  else {
    command = "p0";
    pnh_.setParam("pheumatic", false);
  }
  ROS_INFO("Receive pheumatic command");
  mySerial.write(command);
  return true;
}

bool ArduinoControl::check_suck_success(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  mySerial.write("s");
  ros::Duration(0.1).sleep();
  auto result = mySerial.readline();
  int voltage = atoi(result.c_str());
  ROS_INFO("Voltage received: %d", voltage);
  if(voltage >= vacuum_thres)
    res.success = true;
  else res.success = false;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arduino_control");
  ros::NodeHandle nh, pnh("~");
  ArduinoControl foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
