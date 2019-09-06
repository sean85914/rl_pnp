#include <cstdio> // printf
#include <iostream> // std::cout
#include <iomanip>
#include <fstream> // std::ifstream
#include <Eigen/Dense> // Eigen core, SVD
#include <tf/tf.h> // Transformation

const int MIN_DATA_AMOUNT = 4; // At least 4 data points is required
/*
 *  Get transformation from two point sets in given file using point registration for hand-eye calibration
 *  
 *  Editor: Sean Lu
 *  Last edited: 8/2, 2019
 *  Changelog:
 *    2019/8/2: fix matrix multiplication error, ignore empty lines, exist if too few data points
 */

/*
 *  Parse file and return target and source point sets
 *  [param]in char* file: file name
 *  [param]in Eigen::MatrixXf &target: target point set matrix reference
 *  [param]in Eigen::MatrixXf &source: source point set matrix reference
 *  [param]out: 1 if successfully load file, 0 otherwise
 */ 
bool parse_file(char*, Eigen::MatrixXf&, Eigen::MatrixXf&);
/*
 *  Compute the best transformation from given two sets
 *  Process: 
 *    1. Compute centroids
 *    2. Compute deviation
 *    3. Compute covariance matrix of two sets
 *    4. SVD among covariance and get U, V matrices
 *    5. Compute rotation matrix and translation, refer to: https://blog.csdn.net/zhouyelihua/article/details/77807541
 *  [param]in Eigen::MatrixXf &target: target point set matrix reference
 *  [param]in Eigen::MatrixXf &source: source point set matrix reference
 *  [param]in tf::Transform &t: transformation placeholder
 */
void compute(Eigen::MatrixXf&, Eigen::MatrixXf&, tf::Transform&);
/*
 *  Get best transformation from given file, will call \parse_file\ then \compute\
 *  [param]in char* file: file name
 *  [param]in tf::Transform &t: transformation placeholder
 *  [param]out bool: 1 if successfully compute the result, 0 otherwise
 */
bool get_best_transformation(char*, tf::Transform&);

int main(int argc, char** argv)
{
  if(argc!=2) {
    printf("Not enough input, please provide input file...\n");
    return -1;
  }
  tf::Transform T;
  if(!get_best_transformation(argv[1], T)) return -1;
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
  return 0;
}

bool parse_file(char *file, Eigen::MatrixXf &target, Eigen::MatrixXf &source){
  std::ifstream f;
  std::string line;
  int count = 0;
  f.open(file, std::ifstream::in);
  if(!f){
    printf("\033[1;31mCannot open file! Abort...\n\033[0m");
    return 0;
  }
  while(std::getline(f, line)) {if(!line.empty()) ++count; else {printf("\033[1;33mGot empty line, ignore...\033[0m\n");}}
  if(count<MIN_DATA_AMOUNT) {
    printf("\033[1;31mNot enough data points, at least four is required! Existing...\033[0m\n"); 
    return 0;
  }
  target = Eigen::MatrixXf(count, 3);
  source = Eigen::MatrixXf(count, 3);
  // return the cursor to the begining of the file
  f.clear();
  f.seekg(0, std::ios::beg);
  int row = 0;
  while(!f.eof() && row<count){
    std::getline(f, line);
    std::stringstream ss(line);
    int i = 0;
    while(ss.good() && i < 6){
      double val;
      switch(i){
        case 0: case 1: case 2:
          ss >> val; target(row, i) = val; break;
        case 3: case 4: case 5:
          ss >> val; source(row, i-3) = val; break;
      } ++i;
    } ++row;
  }
  return 1;
}

void compute(Eigen::MatrixXf &target, Eigen::MatrixXf &source, tf::Transform &t){
  // Compute centroid
  Eigen::Vector3f target_centroid, source_centroid;
  double target_sum = 0, source_sum = 0;
  for(int i=0; i<3; ++i){
    for(int j=0; j<target.rows(); ++j){
      target_sum += target(j, i);
      source_sum += source(j, i);
    }
    target_centroid(i) = target_sum/(double)target.rows();
    source_centroid(i) = source_sum/(double)source.rows();
    target_sum = source_sum = 0;
  }
  // Compute deviation
  Eigen::MatrixXf target_deviation(target.rows(), 3),
                  source_deviation(source.rows(), 3);
  for(int i=0; i<target.rows(); ++i){
    for(int j=0; j<3; ++j){
      target_deviation(i, j) = target(i, j) - target_centroid(j);
      source_deviation(i, j) = source(i, j) - source_centroid(j);
    }
  }
  // Compute H
  Eigen::MatrixXf H(3, 3); H = source_deviation.transpose() * target_deviation;
  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Compute rotation and translation
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
  Eigen::Array3f trans = target_centroid - R * source_centroid;
  tf::Matrix3x3 rot(R(0, 0), R(0, 1), R(0, 2),
                    R(1, 0), R(1, 1), R(1, 2),
                    R(2, 0), R(2, 1), R(2, 2));
  tf::Vector3 vec(trans(0), trans(1), trans(2));
  t = tf::Transform(rot, vec);
  // Eigen::Matrix4f
  Eigen::Matrix4f homo = Eigen::Matrix4f::Zero();
  homo.block<3, 3>(0, 0) = R; homo.block<3, 1>(0, 3) = trans; homo(3, 3) = 1.0;
  std::cout.setf(std::ios::showpoint); std::cout.setf(std::ios::fixed, std::ios::floatfield);
  std::cout << "Homogeneous transformation matrix: \n" << std::setprecision(6) << homo << "\n";
  std::cout << "----------------------------------------------------\n";
  double sum_err_square = 0.0f;
  Eigen::MatrixXf source_transformed(target.rows(), 3);
  std::cout << "Registration error: \n";
  for(int i=0; i<target.rows(); ++i){
    for(int j=0; j<3; ++j){
      source_transformed(i, j) = R(j, 0)*source(i, 0) + R(j, 1)*source(i, 1) + R(j, 2)*source(i, 2) + trans(j);
    }
  }
  for(int i=0; i<target.rows(); ++i){
    std::cout.setf(std::ios::showpoint); std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout << std::setprecision(6) 
              << "[" << source(i, 0) << ", " << source(i, 1) << ", " << source(i, 2) << "]\t -> " 
              << "[" << source_transformed(i, 0) << ", " << source_transformed(i, 1) << ", " << source_transformed(i, 2) << "]: \t"
              << "[" << target(i, 0) << ", " << target(i, 1) << ", " << target(i, 2) << "]" << " \tError: ";
    double err_x = target(i, 0) - source_transformed(i, 0);
    double err_y = target(i, 1) - source_transformed(i, 1);
    double err_z = target(i, 2) - source_transformed(i, 2);
    double term_err = err_x*err_x + err_y*err_y + err_z*err_z;
    std::cout << term_err << "\n";
    sum_err_square += term_err;
  }
  std::cout << "\n\nMSE: " << sum_err_square/target.rows() << "\n";
}

bool get_best_transformation(char* file, tf::Transform& t){
  Eigen::MatrixXf target, source;
  if(!parse_file(file, target, source)) return 0;
  compute(target, source, t);
  return 1;
}
