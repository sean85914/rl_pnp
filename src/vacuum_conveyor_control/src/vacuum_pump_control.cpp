// STD
#include <experimental/filesystem>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// SRV
#include <std_srvs/SetBool.h>
// Serial
#include <serial/serial.h>

class ArduinoControl{
 private:
  bool is_upper; // Deciding type, true means that if suck success, the value reported from the sensor will be higher than the threshold, false otherwise
  int baudrate;
  int vacuum_thres_tool_3; // Value from sensor higher/lower than this value will consider as fail, for tool 3 (small suction cup)
  int vacuum_thres_tool_2; // Value from sensor higher/lower than this value will consider as fail, for tool 2 (bag cup suction cup)
  const int TO = 50; // Serial timeout, not sure what is this
  std::string port;
  serial::Serial mySerial;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer vacuum_srv;
  ros::ServiceServer check_success_srv;
  ros::Timer checkParameterTimer;
  bool vacuum_control_cb(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  bool check_suck_success(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  void checkParameterTimerCallback(const ros::TimerEvent& event);
 public:
  ArduinoControl(ros::NodeHandle, ros::NodeHandle);
  ~ArduinoControl();
};

ArduinoControl::ArduinoControl(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), baudrate(115200){
  // Get parameters
  if(!pnh_.getParam("is_upper", is_upper)) is_upper = true;
  if(!pnh_.getParam("port", port)) port = "/dev/ttyACM0";
  if(!pnh_.getParam("vacuum_thres_tool_2", vacuum_thres_tool_2)) vacuum_thres_tool_2 = 350;
  if(!pnh_.getParam("vacuum_thres_tool_3", vacuum_thres_tool_3)) vacuum_thres_tool_3 = 465;
  // Show parameters
  ROS_INFO("[%s] port: %s", ros::this_node::getName().c_str(), port.c_str());
  ROS_INFO("[%s] vacuum_thres_tool_2: %d", ros::this_node::getName().c_str(), vacuum_thres_tool_2);
  ROS_INFO("[%s] vacuum_thres_tool_3: %d", ros::this_node::getName().c_str(), vacuum_thres_tool_3);
  ROS_INFO("[%s] type: %s", ros::this_node::getName().c_str(), (is_upper?"upper":"lower"));
  checkParameterTimer = pnh_.createTimer(ros::Duration(1.0), &ArduinoControl::checkParameterTimerCallback, this);
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
  check_success_srv = pnh_.advertiseService("check_suck_success", &ArduinoControl::check_suck_success, this);
}

ArduinoControl::~ArduinoControl(){
  std::string node_ns = pnh_.getNamespace(),
              package_path = ros::package::getPath("vacuum_conveyor_control"),
              yaml_path = package_path + "/config/vacuum_data.yaml",
              cmd_str = "rosparam dump " + yaml_path + " " + node_ns;
  std::experimental::filesystem::path path(package_path+"/config");
  if(!std::experimental::filesystem::exists(path)){
    std::experimental::filesystem::create_directory(path);
  }
  system(cmd_str.c_str());
  mySerial.write("c");
  mySerial.close();
}

bool ArduinoControl::vacuum_control_cb(std_srvs::SetBool::Request  &req, 
                                       std_srvs::SetBool::Response &res){
  std::string command = (req.data?"o":"c");
  ROS_INFO("Receive vacuum command: turn %s", (req.data?"on":"off"));
  mySerial.write(command);
  return true;
}

bool ArduinoControl::check_suck_success(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  mySerial.write("s");
  ros::Duration(0.1).sleep();
  auto result = mySerial.readline();
  int voltage = atoi(result.c_str());
  int curr_tool_id;
  if(!nh_.getParam("/curr_tool_id", curr_tool_id)){
    ROS_ERROR("No current tool id parameter set, exit...");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
  int vacuum_thres;
  if(curr_tool_id==2) vacuum_thres = vacuum_thres_tool_2;
  else if(curr_tool_id==3) vacuum_thres = vacuum_thres_tool_3;
  else{
    ROS_ERROR("Current tool is not suction cup, abort request...");
    return true;
  }
  ROS_INFO("Voltage received: %d", voltage);
  if(is_upper){
    if(voltage >= vacuum_thres)
      res.success = true;
    else res.success = false;
  }else{ // lower
    if(voltage <= vacuum_thres)
      res.success = true;
    else res.success = false;
  }
  return true;
}

void ArduinoControl::checkParameterTimerCallback(const ros::TimerEvent& event){
  int tmp; 
  if(pnh_.getParam("vacuum_thres_tool_2", tmp)){
    if(tmp!=vacuum_thres_tool_2){
      ROS_INFO("[%s] vacuum_thres_tool_2 changed from %d to %d", ros::this_node::getName().c_str(), vacuum_thres_tool_2, tmp);
      vacuum_thres_tool_2 = tmp;
    }
  }
  if(pnh_.getParam("vacuum_thres_tool_3", tmp)){
    if(tmp!=vacuum_thres_tool_3){
      ROS_INFO("[%s] vacuum_thres_tool_3 changed from %d to %d", ros::this_node::getName().c_str(), vacuum_thres_tool_3, tmp);
      vacuum_thres_tool_3 = tmp;
    }
  }
  bool tmp_bool; 
  if(pnh_.getParam("is_upper", tmp_bool)){
    if(tmp_bool!=is_upper){
      std::string first  = (is_upper?"upper":"lower"),
                  second = (tmp_bool?"upper":"lower");
      ROS_INFO("[%s] Vacuum type changes from %s to %s", ros::this_node::getName().c_str(), first.c_str(), second.c_str());
      is_upper = tmp_bool;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arduino_control");
  ros::NodeHandle nh, pnh("~");
  ArduinoControl foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
