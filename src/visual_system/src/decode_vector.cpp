/*
  Read given PCD file, and get the position and yaw angle w.r.t. robot coordinate which the agent selected to operate
  Wrap as a Python-callable C++ interface:
  '''
  >>> from libconversion import conversion
  >>> filename = ...
  >>> u = ... # in range [0, 224)
  >>> v = ... # in range [0, 224)
  >>> index = ... # in range [0, 6)
  >>> conversion(filename, u, v, index)
  {'x': ..., 'y': ..., 'z': ..., 'yaw': ...}
  '''
  The selected action can be get from saved CSV file and has the form [index, u, v] where:
    * index: primitive index, 0 for small suction cup, 1 for medium suction cup, 2, 3, 4 and 5 for parallel-jaw gripper
             with yaw -90, -45, 0 and 45 degree respectively
    * u: pixel raw
    * v: pixel column
 */

// STL
#include <cassert> // assert
#include <tuple> // std::tuple
#include <utility> // std::pair
#include <vector> // std::vector
// Boost
#include <boost/python.hpp>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace boost::python;

// Primitive number
const int NUM_OF_PRIMITIVE = 6;
// Predefined workspace range
const double X_LOWER = 0.72f;
const double Y_LOWER = -0.54f;
const double X_UPPER = 1.02f;
const double Y_UPPER = -0.24f;
// Heightmap resolution
const int RESOLUTION = 224;

// Self-defined type
typedef std::pair<int, int> pixel;
typedef std::tuple<double, double, double> coord;

dict conversion(std::string filename, int u_, int v_, int index)
{
  dict dictionary;
  if((u_<0||u_>=RESOLUTION)||(v_<0||v_>=RESOLUTION)||(index<0||index>NUM_OF_PRIMITIVE)){
    std::cerr << "\033[1;31mOut of range\033[0m\n";
    return dictionary;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if(pcl::io::loadPCDFile(filename, *cloud)==-1){
    std::cerr << "\033[1;31mCan't read given PCD\033[0m\n";
    return dictionary;
  }
  std::vector<std::vector<coord>> map(RESOLUTION);
  for(int i=0; i<RESOLUTION; ++i){
    map[i] = std::vector<coord>(RESOLUTION, coord(0.0f, 0.0f, 0.0f));
  }
  for(auto p: cloud->points){
    int u = floor((X_UPPER-p.x)/(X_UPPER-X_LOWER)*RESOLUTION),
        v = floor((Y_UPPER-p.y)/(Y_UPPER-Y_LOWER)*RESOLUTION);
    map[u][v] = coord(p.x, p.y, p.z);
  }
  coord res = map[u_][v_];
  
  std::cout << std::get<0>(res) << " "
            << std::get<1>(res) << " "
            << std::get<2>(res) << "\n";
  
  dictionary["x"] = std::get<0>(res);
  dictionary["y"] = std::get<1>(res);
  dictionary["z"] = std::get<2>(res);
  if(index==0||index==1)
    dictionary["yaw"] = 0.0;
  else{
    dictionary["yaw"] = -M_PI/2+M_PI/4*(index-2);
  }
  return dictionary;
}

BOOST_PYTHON_MODULE(libconversion)
{
  def("conversion", conversion, (arg("filename"), arg("u_"), arg("v_")));
}
