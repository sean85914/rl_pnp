#include <ros/ros.h>
#include <ros/package.h>
#include <tinyxml2.h>

int main(int argc, char** argv)
{
  // ./set_urdf [x] [y] [z] [roll] [pitch] [yaw]
  if(argc!=7){
    std::cout << "\033[1;31mNot enough input arguments, abort...\033[0m\n";
    std::cout << "Example usage: ./set_urdf [x] [y] [z] [roll] [yaw] [pitch]\n";
    exit(EXIT_FAILURE);
  }
  double x = atof(argv[1]),
         y = atof(argv[2]),
         z = atof(argv[3]),
         roll = atof(argv[4]),
         pitch = atof(argv[5]),
         yaw = atof(argv[6]);
  std::string package_path = ros::package::getPath("robot_description");
  std::string urdf_path = package_path + "/urdf/system.urdf.xacro";
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError load_error = doc.LoadFile(urdf_path.c_str());
  if(load_error!=tinyxml2::XML_SUCCESS){
    std::cout << "\033[1;31mCan't open" << urdf_path <<" , abort...\033[0m\n";
    exit(EXIT_FAILURE);
  }
  /*
   *  - robot
   *    ...
   *    - joint ...
   *      - origin xyz rpy
   */
  tinyxml2::XMLElement *root = doc.RootElement(),
                       *joint = root->FirstChildElement("joint"),
                       *origin = joint->FirstChildElement("origin");
  if(root==NULL or joint==NULL or origin==NULL){
    std::cout << "Can't parse XARCO correctly, abort\033[0m\n";
    exit(EXIT_FAILURE);
  }
  std::string xyz_string = std::to_string(x)    + " " + std::to_string(y)     + " " + std::to_string(z),
              rpy_string = std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw);
  std::cout << origin->Attribute("xyz") << "\n" << origin->Attribute("rpy") << "\n";
  std::cout << "xyz set from: [" << origin->Attribute("xyz") << "] to [" << xyz_string << "]\n"
            << "rpy set from: [" << origin->Attribute("rpy") << "] to [" << rpy_string << "]\n";
  origin->SetAttribute("xyz", xyz_string.c_str());
  origin->SetAttribute("rpy", rpy_string.c_str());
  doc.SaveFile(urdf_path.c_str());
  return 0;
}
