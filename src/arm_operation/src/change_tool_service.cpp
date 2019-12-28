#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <abb_node/robot_SetJoints.h>
#include <abb_node/robot_SetCartesian.h>
#include <abb_node/robot_GetJoints.h>
#include <abb_node/robot_GetCartesian.h>
#include <abb_node/robot_SetSpeed.h>
#include <abb_node/robot_SetZone.h>
#include <arm_operation/change_tool.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

inline std::vector<double> deg2rad(const std::vector<double> in){
  assert(in.size()==6);
  std::vector<double> res;
  for(int i=0; i<6; ++i){
    double rad = in[i]*M_PI/180.0;
    res.push_back(rad);
  }
  return res;
}

inline bool in_range(double data, double upper, double lower){
  if(data>upper or data<lower) return false;
  else return true;
}

class ChangeToolService{
 private:
  apriltags_ros::AprilTagDetectionArray detections;
  std::vector<std::vector<double>> joints_vector;
  ros::NodeHandle nh_, pnh_;
  tf::TransformListener listener;
  ros::ServiceServer change_tool_service;
  ros::ServiceClient getCartesian,
                     getJoints,
                     setCartesian,
                     setJoints,
                     setSpeed,
                     setZone;
  void printInfo(void){
    for(auto x: joints_vector){
      for(auto data: x){
        std::cout << data << " ";
      } std::cout << "\n";
    }
  }
  void setupParameters(void){
    bool in_radian;
    pnh_.getParam("in_radian", in_radian);
    std::vector<double> arr;
    if(!pnh_.getParam("a_1", arr)){
      ROS_ERROR("Can't get a_1, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("a_2", arr)){
      ROS_ERROR("Can't get a_2, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("a_3", arr)){
      ROS_ERROR("Can't get a_3, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("b_1", arr)){
      ROS_ERROR("Can't get b_1, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("b_2", arr)){
      ROS_ERROR("Can't get b_2, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("b_3", arr)){
      ROS_ERROR("Can't get b_3, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("c_1", arr)){
      ROS_ERROR("Can't get c_1, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("c_2", arr)){
      ROS_ERROR("Can't get c_2, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    if(!pnh_.getParam("c_3", arr)){
      ROS_ERROR("Can't get c_3, exit...");
      exit(EXIT_FAILURE);
    } joints_vector.push_back((in_radian?arr:deg2rad(arr)));
    printInfo();
  }
  bool service_cb(arm_operation::change_tool::Request  &req,
                  arm_operation::change_tool::Response &res){
    if((!in_range(req.now, 3, 1) or !in_range(req.togo, 3, 1)) and !req.now==-1){
      res.result = "request tool out of range";
      return true;
    } else if(req.now == req.togo){
      res.result = "same tool, abort request";
      return true;
    }
    ros::Time ts = ros::Time::now();
    std::string info = "Receive new request: ";
    if(req.now == -1){
      info += "to " + std::to_string(req.togo);
    } else{
      info += "from " + std::to_string(req.now) + " to " + std::to_string(req.togo);
    }
    ROS_INFO("%s", info.c_str());
    abb_node::robot_GetJoints get_joints_srv;
    abb_node::robot_SetJoints set_joints_srv;
    abb_node::robot_GetCartesian get_cartesian_srv;
    abb_node::robot_SetCartesian set_cartesian_srv;
    abb_node::robot_SetSpeed set_speed_srv;
    abb_node::robot_SetZone set_zone_srv;
    set_joints_srv.request.position.resize(6); // Seg. fault if not resize to correct size  
    // Set zone to Z1
    set_zone_srv.request.mode = 2;
    setZone.call(set_zone_srv);
    // Set speed to (400, 150)
    set_speed_srv.request.tcp = 400.0f;
    set_speed_srv.request.ori = 150.0f;
    setSpeed.call(set_speed_srv);
    // Get current joints and put it into buffer
    getJoints.call(get_joints_srv);
    std::vector<double> original_pose{get_joints_srv.response.j1, 
                                      get_joints_srv.response.j2, 
                                      get_joints_srv.response.j3, 
                                      get_joints_srv.response.j4, 
                                      get_joints_srv.response.j5, 
                                      get_joints_srv.response.j6};
    // Put `now` to its home
    if(req.now!=-1){
      std::copy(joints_vector[(req.now-1)*3].begin(), joints_vector[(req.now-1)*3].end(), set_joints_srv.request.position.begin());
      setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
      std::copy(joints_vector[(req.now-1)*3+1].begin(), joints_vector[(req.now-1)*3+1].end(), set_joints_srv.request.position.begin());
      setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
      std::copy(joints_vector[(req.now-1)*3+2].begin(), joints_vector[(req.now-1)*3+2].end(), set_joints_srv.request.position.begin());
      setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
    }
    // Take `togo` away
    std::copy(joints_vector[(req.togo-1)*3+2].begin(), joints_vector[(req.togo-1)*3+2].end(), set_joints_srv.request.position.begin());
    setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
    std::copy(joints_vector[(req.togo-1)*3+1].begin(), joints_vector[(req.togo-1)*3+1].end(), set_joints_srv.request.position.begin());
    setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
    // Get current pose and back 50 mm among X-axis
    getCartesian.call(get_cartesian_srv);
    set_cartesian_srv.request.cartesian.push_back(get_cartesian_srv.response.x-50.0);
    set_cartesian_srv.request.cartesian.push_back(get_cartesian_srv.response.y);
    set_cartesian_srv.request.cartesian.push_back(get_cartesian_srv.response.z);
    set_cartesian_srv.request.quaternion.push_back(get_cartesian_srv.response.q0);
    set_cartesian_srv.request.quaternion.push_back(get_cartesian_srv.response.qx);
    set_cartesian_srv.request.quaternion.push_back(get_cartesian_srv.response.qy);
    set_cartesian_srv.request.quaternion.push_back(get_cartesian_srv.response.qz);
    setCartesian.call(set_cartesian_srv); ros::Duration(0.5).sleep();
    // Return to original pose
    std::copy(original_pose.begin(), original_pose.end(), set_joints_srv.request.position.begin());
    setJoints.call(set_joints_srv); ros::Duration(0.5).sleep();
    res.result = "success";
    // Set speed back to (200, 100)
    set_speed_srv.request.tcp = 200.0f;
    set_speed_srv.request.ori = 100.0f;
    setSpeed.call(set_speed_srv);
    // Set zone back to Z0
    set_zone_srv.request.mode = 1;
    setZone.call(set_zone_srv);
    double time_response = (ros::Time::now()-ts).toSec();
    ROS_INFO("Service response complete, takes %f seconds", time_response);
    // Make gripper in zero degree
    if(req.togo==1){
      // Save current cartesian position
      getCartesian.call(get_cartesian_srv);
      double original_x = get_cartesian_srv.response.x,
             original_y = get_cartesian_srv.response.y,
             original_z = get_cartesian_srv.response.z;
      ROS_INFO("Save position: [%f, %f, %f]", original_x, original_y, original_z);
      set_cartesian_srv.request.cartesian[0] = original_x;
      set_cartesian_srv.request.cartesian[1] = original_y;
      set_cartesian_srv.request.cartesian[2] = original_z - 85.0;
      set_cartesian_srv.request.quaternion[0] = get_cartesian_srv.response.q0;
      set_cartesian_srv.request.quaternion[1] = get_cartesian_srv.response.qx;
      set_cartesian_srv.request.quaternion[2] = get_cartesian_srv.response.qy;
      set_cartesian_srv.request.quaternion[3] = get_cartesian_srv.response.qz;
      setCartesian.call(set_cartesian_srv); ros::Duration(0.6).sleep();
      while(1){
        // Check if detected
        ros::Time ts = ros::Time::now();
        apriltags_ros::AprilTagDetectionArrayConstPtr detections;
        while((ros::Time::now()-ts).toSec()<=2.0){
          detections = ros::topic::waitForMessage<apriltags_ros::AprilTagDetectionArray>("/tag_detections", ros::Duration(1.0));
          if(detections->detections.size()!=0) break;
        }
        if(detections->detections.size()!=0) break;
        // Get current joints and put it into buffer
        getJoints.call(get_joints_srv);
        ROS_INFO("Rotating joint 6 -15 degree...");
        set_joints_srv.request.position[0] = get_joints_srv.response.j1;
        set_joints_srv.request.position[1] = get_joints_srv.response.j2;
        set_joints_srv.request.position[2] = get_joints_srv.response.j3;
        set_joints_srv.request.position[3] = get_joints_srv.response.j4;
        set_joints_srv.request.position[4] = get_joints_srv.response.j5;
        set_joints_srv.request.position[5] = get_joints_srv.response.j6 - 15.0/180.0*M_PI;
        if(set_joints_srv.request.position[5]<=-400.0/180.0*M_PI){
          ROS_ERROR("Over joint6 max range, failed");
          return true;
        }
        setJoints.call(set_joints_srv); ros::Duration(0.6).sleep();
      } // End while
      tf::StampedTransform stf;
      for(int i=0; i<2; ++i){ // Do twice
        try{
          listener.waitForTransform("base_link", "gripper", ros::Time(0), ros::Duration(1.0f));
          listener.lookupTransform("base_link", "gripper", ros::Time(0), stf);
          tf::Matrix3x3 gripper_mat(stf.getRotation());
          tf::Vector3 gripper_z = gripper_mat.getColumn(2), base_neg_x(-1.0f, 0.0f, 0.0f);
          double theta = gripper_z.angle(base_neg_x); if(theta<0.0) theta = -theta;
          getJoints.call(get_joints_srv);
          double theta_deg = theta*180.0/M_PI;
          ROS_INFO("Rotating joint 6 %f degree...", theta_deg);
          set_joints_srv.request.position[0] = get_joints_srv.response.j1;
          set_joints_srv.request.position[1] = get_joints_srv.response.j2;
          set_joints_srv.request.position[2] = get_joints_srv.response.j3;
          set_joints_srv.request.position[3] = get_joints_srv.response.j4;
          set_joints_srv.request.position[4] = get_joints_srv.response.j5;
          set_joints_srv.request.position[5] = get_joints_srv.response.j6 - theta;
          setJoints.call(set_joints_srv); ros::Duration(0.3).sleep();
        }
        catch(tf::TransformException &ex){
          ROS_ERROR("%s", ex.what());
        }
      }
      getCartesian.call(get_cartesian_srv);
      set_cartesian_srv.request.cartesian[0] = original_x;
      set_cartesian_srv.request.cartesian[1] = original_y;
      set_cartesian_srv.request.cartesian[2] = original_z;
      set_cartesian_srv.request.quaternion[0] = get_cartesian_srv.response.q0;
      set_cartesian_srv.request.quaternion[1] = get_cartesian_srv.response.qx;
      set_cartesian_srv.request.quaternion[2] = get_cartesian_srv.response.qy;
      set_cartesian_srv.request.quaternion[3] = get_cartesian_srv.response.qz;
      setCartesian.call(set_cartesian_srv); ros::Duration(0.3).sleep();
    }
    return true;
  }
 public:
  ChangeToolService(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
    setupParameters();
    change_tool_service = pnh_.advertiseService("change_tool_service", &ChangeToolService::service_cb, this);
    std::vector<std::string> service_name{"/abb/GetCartesian",
                                          "/abb/GetJoints",
                                          "/abb/SetCartesian",
                                          "/abb/SetJoints",
                                          "/abb/SetSpeed",
                                          "/abb/SetZone"};
    for(auto name: service_name){
      while(!ros::service::waitForService(name, ros::Duration(3.0)) and ros::ok()){
        ROS_WARN("Waiting for service: %s", name.c_str());
      }
    }
    getCartesian = nh_.serviceClient<abb_node::robot_GetCartesian>(service_name[0]);
    getJoints = nh_.serviceClient<abb_node::robot_GetJoints>(service_name[1]);
    setCartesian = nh_.serviceClient<abb_node::robot_SetCartesian>(service_name[2]);
    setJoints = nh_.serviceClient<abb_node::robot_SetJoints>(service_name[3]);
    setSpeed = nh_.serviceClient<abb_node::robot_SetSpeed>(service_name[4]);
    setZone = nh_.serviceClient<abb_node::robot_SetZone>(service_name[5]);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "change_tool_service");
  ros::NodeHandle nh, pnh("~");
  ChangeToolService foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
