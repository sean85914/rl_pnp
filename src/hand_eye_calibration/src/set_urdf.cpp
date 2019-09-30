#include <ros/ros.h>
#include <ros/package.h>
#include <tinyxml2.h>

int main(int argc, char** argv)
{
  // ./set_urdf [x] [y] [z] [roll] [pitch] [yaw]
  if(argc!=8){
    std::cout << "\033[1;31mNot enough input arguments, abort...\033[0m\n";
    std::cout << "Example usage: ./set_urdf [joint_name] [x] [y] [z] [roll] [yaw] [pitch]\n";
    exit(EXIT_FAILURE);
  }
  // INFO
  double x = atof(argv[2]),
         y = atof(argv[3]),
         z = atof(argv[4]),
         roll = atof(argv[5]),
         pitch = atof(argv[6]),
         yaw = atof(argv[7]);
  std::cout << "Info:\n" 
            << "\tJoint name: " << argv[1] << "\n"
            << "\tx: "     << x << "\n"
            << "\ty: "     << y << "\n"
            << "\tz: "     << z << "\n"
            << "\troll: "  << roll << "\n"
            << "\tpitch: " << pitch << "\n"
            << "\tyaw: "   << yaw << "\n";
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
                       *origin;
  if(root==NULL or joint==NULL){
    std::cout << "\033[1;31mCan't parse XARCO correctly, abort\033[0m\n";
    exit(EXIT_FAILURE);
  }
  while(strcmp(joint->Attribute("name"), argv[1])!=0){
    joint = joint->NextSiblingElement("joint");
    if(joint==NULL) break;
  }
  if(joint==NULL){
    std::cout << "\033[1;31mCan't parse XARCO correctly, abort\033[0m\n";
    exit(EXIT_FAILURE);
  }
  origin = joint->FirstChildElement("origin");
  std::string xyz_string = std::to_string(x)    + " " + std::to_string(y)     + " " + std::to_string(z),
              rpy_string = std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw);
  std::cout << "xyz set from: [" << origin->Attribute("xyz") << "] to [" << xyz_string << "]\n"
            << "rpy set from: [" << origin->Attribute("rpy") << "] to [" << rpy_string << "]\n";
  origin->SetAttribute("xyz", xyz_string.c_str());
  origin->SetAttribute("rpy", rpy_string.c_str());
  doc.SaveFile(urdf_path.c_str());
  return 0;
}
