#include <point_set_registration.h>

bool PointSetRegistration::parse_file(const std::string file){
  // Check if file exist
  std::ifstream f;
  std::string line;
  int count = 0;
  f.open(file, std::ifstream::in);
  if(!f){
    printf("\033[1;31mCan't open file! Abort...\033[0m\n");
    return false;
  }
  // Get number of data
  while(std::getline(f, line)) {
    if(!line.empty()) ++count; 
    else printf("\033[1;33mGit empty line, ignore...\033[0m\n");
  }
  if(count<MIN_DATA){
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
    while(ss.good() && i<6){
      double val;
      switch(i){
        case 0: case 1: case 2:
          ss >> val; target(row, i) = val; break;
        case 3: case 4: case 5:
          ss >> val; source(row, i-3) = val; break;
      } ++i;
    } ++row;
  }
  return true;
}
void PointSetRegistration::compute(void){
  // Reference: Least-Squares Fitting of Two 3-D Point Sets
  // 1. Compute centroid
  Eigen::Vector3f target_centroid, source_centroid;
  double target_sum = 0.0, source_sum = 0.0;
  for(int i=0; i<3; ++i){
    for(int j=0; j<target.rows(); ++j){
      target_sum += target(j, i);
      source_sum += source(j, i);
    }
    target_centroid(i) = target_sum/(double)target.rows(); // (4)
    source_centroid(i) = source_sum/(double)source.rows(); // (6)
    target_sum = source_sum = 0;
  }
  // 2. Compute deviation
  Eigen::MatrixXf target_deviation(target.rows(), 3),
                  source_deviation(source.rows(), 3);
  for(int i=0; i<target.rows(); ++i){
    for(int j=0; j<3; ++j){
      target_deviation(i, j) = target(i, j) - target_centroid(j); // (8)
      source_deviation(i, j) = source(i, j) - source_centroid(j); // (7)
    }
  }
  // 3. Compute H
  Eigen::MatrixXf H(3, 3); H = source_deviation.transpose() * target_deviation; // (11)
  // 4. SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV); // (12)
  // 5. Compute rotation and translation
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose(); // (13)
  Eigen::Array3f trans = target_centroid - R * source_centroid; // (10)
  // Convert to tf::Transform
  tf::Matrix3x3 rot(R(0, 0), R(0, 1), R(0, 2),
                    R(1, 0), R(1, 1), R(1, 2),
                    R(2, 0), R(2, 1), R(2, 2));
  tf::Vector3 vec(trans(0), trans(1), trans(2));
  result_transform = tf::Transform(rot, vec);
  // Compute error
  double sum_err_square = 0.0f;
  Eigen::MatrixXf source_transformed(target.rows(), 3);
  for(int i=0; i<target.rows(); ++i){
    for(int j=0; j<3; ++j){
      source_transformed(i, j) = R(j, 0)*source(i, 0) + R(j, 1)*source(i, 1) + R(j, 2)*source(i, 2) + trans(j);
    }
  }
  for(int i=0; i<target.rows(); ++i){
    double err_x = target(i, 0) - source_transformed(i, 0);
    double err_y = target(i, 1) - source_transformed(i, 1);
    double err_z = target(i, 2) - source_transformed(i, 2);
    double term_err = err_x*err_x + err_y*err_y + err_z*err_z;
    sum_err_square += term_err;
  }
  error = sqrt(sum_err_square/target.rows())*1000; // error in millimeter
}
PointSetRegistration::PointSetRegistration(const data_vector input_data_vec): error(0.0){
  int count = input_data_vec.size();
  if(count<MIN_DATA){
    printf("\033[1;31mNot enough data points, at least four is required! Existing...\033[0m\n");
    return;
  }
  target = Eigen::MatrixXf(count, 3);
  source = Eigen::MatrixXf(count, 3);
  int row = 0;
  for(auto data: input_data_vec){
    geometry_msgs::Point hand_data = data.first;
    geometry_msgs::Point cam_data  = data.second;
    for(int i=0; i<3; ++i){
      switch(i){
        case 0:
          target(row, i) = hand_data.x;
          source(row, i) = cam_data.x;
          break;
        case 1:
          target(row, i) = hand_data.y;
          source(row, i) = cam_data.y;
          break;
        case 2:
          target(row, i) = hand_data.z;
          source(row, i) = cam_data.z;
          break;
      }
    } ++row;
  }
  compute();
}
