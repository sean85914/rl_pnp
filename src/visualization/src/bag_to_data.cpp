// STD
#include <chrono>
#include <fstream>
// Boost
#include <boost/foreach.hpp>
// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <arm_operation/execution.h>

#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
  // Parse input argument
  bool has_data = false;
  if(argc<3){
    std::cout << "\033[1;31mInsufficient input arguments\n\
Usage: ./bag_to_video bag_name output_video_name [speed]\n\033[0m";
    exit(EXIT_FAILURE);
  }
  std::string in_bag(argv[1]);
  std::string out_name = std::string(argv[2]);
  std::fstream file;
  if(out_name.length()<=4) // .txt
    out_name+=".txt";
  else{
    size_t pos;
    out_name.find(".");
    if(pos==std::string::npos){
      out_name+=".txt";
    }
  }
  std::cout << "Input bag: " << in_bag << "\n";
  std::cout << "Output file: " << out_name << "\n";
  file.open(out_name, std::ios::out);
  // Open bag
  rosbag::Bag bag;
  try{
    bag.open(in_bag, rosbag::bagmode::Read);
  } catch(const rosbag::BagIOException& e){
    std::cout << "\033[1;31mProvided bag name unopenable, please make sure you provide correct name, exiting...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  rosbag::View view(bag);
  double bag_duration = (view.getEndTime() - view.getBeginTime()).toSec();
  auto bag_start_ts = view.getBeginTime();
  std::cout << "Bag duration: " << bag_duration << " seconds\n";
  std::cout << "Writing...\n";
  // Start reading
  auto s_ts = std::chrono::high_resolution_clock::now();
  foreach(const rosbag::MessageInstance m, view){
    if(m.getDataType()=="arm_operation/execution"){
      arm_operation::execution::ConstPtr data_ptr = m.instantiate<arm_operation::execution>();
      has_data = true;
      double time_from_start = (data_ptr->header.stamp-bag_start_ts).toSec();
      file << data_ptr->iteration << "\t" 
           << data_ptr->header.stamp << "\t" 
           << time_from_start << "\t" 
           << data_ptr->primitive << "\t"
           << (data_ptr->is_success?"success":"fail") << "\n";
    }
  }
  if(!has_data){
    std::cout << "\033[1;31mNo data detected, exiting...\n\033[0m";
    file.close();
    exit(EXIT_FAILURE);
  }
  auto e_ts = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(e_ts-s_ts).count();
  file.close();
  std::cout << "Conversion time: " << duration*1e-9 << " seconds\n";
}
