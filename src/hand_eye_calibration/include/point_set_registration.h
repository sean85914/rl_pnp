#ifndef POINT_SET_REGISTRATION_H
#define POINT_SET_REGISTRATION_H

#include <utility> // std::pair
#include <cstdio> // printf
#include <iostream> // std::cout
#include <iomanip>
#include <fstream> // std::ifstream
#include <Eigen/Dense> // Eigen core, SVD
#include <tf/tf.h> // Transformation
#include <geometry_msgs/Point.h>

typedef std::pair<geometry_msgs::Point, geometry_msgs::Point> data_pair;
typedef std::vector<data_pair> data_vector;

class PointSetRegistration{
 private:
  const int MIN_DATA = 4;
  double error;
  Eigen::MatrixXf target;
  Eigen::MatrixXf source;
  tf::Transform result_transform;
  bool parse_file(const std::string file);
  void compute(void);
 public:
  PointSetRegistration(const std::string file): error(0.0){
    if(!parse_file(file)) return;
    compute();
  }
  PointSetRegistration(const data_vector input_data_vec);
  void getData(double &data, tf::Transform &t){
    data = error; t = result_transform;
  }
};

#endif
