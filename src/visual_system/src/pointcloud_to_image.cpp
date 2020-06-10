/*
  Convert given organized point cloud PCD file to both color and depth image
  [Required Input]
    --file, -f: file name for PCD file to be converted
  [Optinal Input]
    --color_dir: directory where color image be saved
    --depth_dir: directory where depth image be saved
    --color_name: saved color image name
    --depth_name: saved depth image name
  [Output]: images saved in `color_dir` and `depth_dir`
 */

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp> // cv::Mat
#include <opencv2/highgui.hpp> // cv::imwrite

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  std::string pcd_filename, color_dir, depth_dir, color_name, depth_name;
  try{
    po::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Convert given organized point cloud PCD file to both color and depth image")
      ("file,f", po::value<std::string>()->required(), "PCD file name")
      ("color_dir", po::value<std::string>(), "Directory for saved color image")
      ("depth_dir", po::value<std::string>(), "Directory for saved depth image")
      ("color_name", po::value<std::string>()->default_value("color.png"), "Name for color image")
      ("depth_name", po::value<std::string>()->default_value("depth.png"), "Name for depth image");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if(vm.count("help")){
      std::cout << desc << "\n";
      return -1;
    }
    po::notify(vm);
    // Get `pcd_filename`
    pcd_filename = vm["file"].as<std::string>();
    // Should extent with `.pcd`
    if(pcd_filename.length()<4 or pcd_filename.substr(pcd_filename.length()-4).compare(".pcd")!=0)
      pcd_filename += ".pcd";
    // Get `color_dir`
    if(vm.count("color_dir")){
      color_dir = vm["color_dir"].as<std::string>()+"/";
    }
    // Get `depth_dir`
    if(vm.count("depth_dir")){
      depth_dir = vm["depth_dir"].as<std::string>()+"/";
    }
    // Get `color_name`
    if(vm.count("color_name")){
      color_name = vm["color_name"].as<std::string>();
      // Should extent with `png`
      if(color_name.length()<4 or color_name.substr(color_name.length()-4).compare(".png")!=0)
        color_name += ".png";
    }
    // Get `depth_name`
    if(vm.count("depth_name")){
      depth_name = vm["depth_name"].as<std::string>();
      if(depth_name.compare(color_name)==0){
        std::cerr << "\033[1;33mGot same name, set `depth_name` to default one...\033[0m\n";
        depth_name = "depth.png";
      }else if(depth_name.length()<4 or depth_name.substr(depth_name.length()-4).compare(".png")!=0)
        // Should extent with `png`
        depth_name += ".png";
    }
  } catch(const po::error &ex){
    std::cerr << ex.what() << "\n";
  }
  if(!color_dir.empty()&&!fs::is_directory(fs::path(color_dir))){
    fs::create_directories(fs::path(color_dir));
  }
  if(!depth_dir.empty()&&!fs::is_directory(fs::path(depth_dir))){
    fs::create_directories(fs::path(depth_dir));
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_filename, *cloud)==-1){
    std::cerr << "\033[1;33mCan't read given PCD file, exit...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  if(!cloud->isOrganized()){
    std::cerr << "\033[1;33mGiven PCD file is unorganized, exit...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  cv::Mat color(cloud->height, cloud->width, CV_8UC3), 
          depth(cloud->height, cloud->width, CV_16UC1);
  for(int i=0; i<cloud->width; ++i){
    for(int j=0; j<cloud->height; ++j){
      color.at<cv::Vec3b>(j, i)[0] = cloud->at(i, j).b;
      color.at<cv::Vec3b>(j, i)[1] = cloud->at(i, j).g;
      color.at<cv::Vec3b>(j, i)[2] = cloud->at(i, j).r;
      depth.at<unsigned short>(j, i) = cloud->at(i, j).z*1000; // meter to millimeter
    }
  }
  cv::imwrite(color_dir+color_name, color);
  cv::imwrite(depth_dir+depth_name, depth);
  return 0;
}
