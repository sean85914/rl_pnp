#include <cstdio> // printf
#include <iostream> // std::cout
#include <tf/tf.h> // Transformation
#include <point_set_registration.h>

int main(int argc, char** argv)
{
  if(argc!=2) {
    printf("Not enough input, please provide input file...\n");
    return -1;
  }
  tf::Transform T;
  double error;
  PointSetRegistration psr(argv[1]);
  psr.getData(error, T);
  double r, p, y;
  T.getBasis().getRPY(r, p, y);
  std::cout << "----------------------------------------------------\n";
  std::cout << "Translation: ["
            << T.getOrigin().getX() << " " << T.getOrigin().getY() << " " << T.getOrigin().getZ() << "]\n"
            << "Orientation: (Quaternion) ["  
            << T.getRotation().getX() << " " << T.getRotation().getY() << " "
            << T.getRotation().getZ() << " " << T.getRotation().getW() << "]\n"
            << "Orientation: (Euler) [" 
            << r << " " << p << " " << y << "]\n";
  std::cout << "Error: " << error << "\n";
  return 0;
}
