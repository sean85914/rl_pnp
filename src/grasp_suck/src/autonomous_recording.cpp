// STD
#include <cstdio>
#include <csignal>
#include <thread>
// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/recorder.h>
// Service
#include <std_srvs/Empty.h>
#include <grasp_suck/recorder.h>

std::string get_longest_common_substring(std::string str_1, std::string str_2){
  std::string res="";
  int min = (str_1.length()>=str_2.length()?str_2.length():str_1.length());
  for(int i=0; i<min; ++i){
    if(str_1[i]==str_2[i])
      res += str_1[i];
  }
  return res;
}

std::string traverse_to_desired_dir(const std::string desired_path, const std::string current_path){
  std::string res="";
  std::string common = get_longest_common_substring(desired_path, current_path);
  std::string curren_path_sub = current_path.substr(common.length(), current_path.length()-common.length());
  std::string desired_path_sub = desired_path.substr(common.length(), desired_path.length()-common.length());
  // From current DIR traverse to common part
  for(int i=curren_path_sub.length()-1; i!=0; --i){
    if(curren_path_sub[i]=='/') res+="../";
  }
  res+="../"+desired_path_sub+"/";
  return res;
}

class Recording{
 public:
  Recording(pid_t pid, ros::NodeHandle nh, ros::NodeHandle pnh): 
    can_record(false), 
    run_episode(0), 
    desired("/home/sean/Documents/exp_bag"),
    nh_(nh), 
    pnh_(pnh)
    {
    topics_to_record.push_back("/record/color/image_raw");
    topics_to_record.push_back("/agent_server_node/execution_info");
    start_recording_service = pnh_.advertiseService("start_recording", &Recording::start_recording_CB, this);
    stop_recording_service  = pnh_.advertiseService("stop_recording", &Recording::stop_recording_CB, this);
    check_record_timer = pnh_.createTimer(ros::Duration(0.1), &Recording::check_record_CB, this);
    char cwd[FILENAME_MAX];
    getcwd(cwd, FILENAME_MAX);
    std::string cwd_string(cwd);
    path_to_desired = traverse_to_desired_dir(desired, cwd_string);
  }
  void recorder_thread(void);
 private:
  bool can_record;
  int run_episode;
  std::string desired;
  std::string path_to_desired;
  std::vector<std::string> topics_to_record;
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer start_recording_service;
  ros::ServiceServer stop_recording_service;
  ros::Timer check_record_timer;
  ros::Time start_record_ts;
  bool start_recording_CB(grasp_suck::recorder::Request&, grasp_suck::recorder::Response&);
  bool stop_recording_CB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(!can_record) return true;
    std::cout << "Bag duration: " << (ros::Time::now()-start_record_ts).toSec() << "\nStop recording...\n";
    kill(getpid(), SIGINT);
    return true;
  }
  void check_record_CB(const ros::TimerEvent&);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_recording_node");
  ros::NodeHandle nh, pnh("~");
  Recording recorder(getpid(), nh, pnh);
  //std::thread record_thread(&Recording::recorder_thread, recorder);
  while(ros::ok()) ros::spinOnce();
  //record_thread.join();
  return 0;
}

/*void Recording::recorder_thread(void){
  while(!can_record){}
  rosbag::RecorderOptions opts;
  opts.prefix = "rl_" + std::to_string(run_episode);
  opts.append_date = true;
  for(auto sub_topic: topics_to_record)
    opts.topics.push_back(sub_topic);
  rosbag::Recorder recorder(opts);
  std::cout << "Start recording...\n";
  int result = recorder.run();
  can_record = false;
}*/

bool Recording::start_recording_CB(grasp_suck::recorder::Request &req, grasp_suck::recorder::Response &res){
  run_episode = req.run_episode;
  can_record = true;
  return true;
}

void Recording::check_record_CB(const ros::TimerEvent &event){
  if(!can_record) return;
  check_record_timer.stop();
  rosbag::RecorderOptions opts;
  opts.prefix = path_to_desired + "rl_e_" + std::to_string(run_episode);
  opts.append_date = false; 
  for(auto sub_topic: topics_to_record)
    opts.topics.push_back(sub_topic);
  rosbag::Recorder recorder(opts);
  ROS_INFO("Start recording...");
  start_record_ts = ros::Time::now();
  int result = recorder.run();
  can_record = false;
}
