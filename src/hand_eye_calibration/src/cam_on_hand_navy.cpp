#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/*
 * Using Navy method to solve hand-eye calibration problem
 * Ref: Robot sensor calibration: solving AX=XB on the Euclidean group
 */
 
inline Eigen::Matrix4f tf2eigen(const tf::Transform t){
  Eigen::Matrix4f res;
  auto rot_mat = t.getBasis();
  auto trans = t.getOrigin();
  res << rot_mat[0].getX(), rot_mat[0].getY(), rot_mat[0].getZ(), trans.getX(),
         rot_mat[1].getX(), rot_mat[1].getY(), rot_mat[1].getZ(), trans.getY(),
         rot_mat[2].getX(), rot_mat[2].getY(), rot_mat[2].getZ(), trans.getZ(),
         0.0f, 0.0f, 0.0f, 1.0f;
  return res;
}
class calibration{
 private:
  bool computed;
  const int REQUIRED_DATA = 4; // At least 3 data points is required
  int num_data; // How many data points we have now
  std::string camera_name; // camera namespace
  std::string tag_name; // tag name
  ros::NodeHandle nh_, pnh_;
  tf::TransformListener listener;
  Eigen::Vector3f trans_X;
  Eigen::Matrix3f matrix_M, rot_mat_X;
  std::vector<tf::Transform> base2ee; // Data placeholder for transformation from robot base to end effector
  std::vector<tf::Transform> cam2tag; // Data placeholder for transformation from camera to tag
  std::vector<Eigen::Matrix4f> A_vec; // Transformation between end effector
  std::vector<Eigen::Matrix4f> B_vec; // Transformation between camera
  std::vector<Eigen::Matrix3f> alpha_vec;
  std::vector<Eigen::Matrix3f> beta_vec;
  void compute_transform(void){
    for(int i=1; i<num_data; ++i){
      Eigen::Matrix3f rot_mat;
      // Get A
      tf::Transform between_ee = base2ee[i].inverse()*base2ee[i-1];
      A_vec.push_back(tf2eigen(between_ee));
      rot_mat = A_vec.back().block<3, 3>(0,0);
      // Get B
      tf::Transform between_cam = cam2tag[i]*cam2tag[i-1].inverse();
      B_vec.push_back(tf2eigen(between_cam));
      rot_mat = B_vec.back().block<3, 3>(0,0);
      // Compute matrix logarithm
      alpha_vec.push_back(rot_mat.log());
      beta_vec.push_back(rot_mat.log());
      // Compute M
      matrix_M += beta_vec.back() * alpha_vec.back().transpose();
    }
    // Compute rotation matrix
    Eigen::Matrix3f trans_mul = matrix_M.transpose()*matrix_M;
    rot_mat_X = trans_mul.sqrt().inverse()*matrix_M.transpose();
    // Compute translation vector
    Eigen::MatrixXf least_square_A(A_vec.size()*3, 3), least_square_b(A_vec.size()*3, 1);
    Eigen::Matrix3f eye_3 = Eigen::Matrix3f::Identity();
    for(int i=0; i<A_vec.size(); ++i){
      least_square_A.block<3, 3>(i*3, 0) = A_vec[i].block<3, 3>(0, 0) - eye_3;
      least_square_b.block<3, 1>(i*3, 0) = \
                  rot_mat_X * B_vec[i].block<3, 1>(0, 3) - A_vec[i].block<3, 1>(0, 3);
    }
    trans_X = (least_square_A.transpose()*least_square_A).inverse()*least_square_A.transpose()*least_square_b;
    // Print
    std::cout << "Rotation matrix: \n" << rot_mat_X << "\n";
    std::cout << "Translation vector: \n" << trans_X << "\n";
    computed = true;
    return;
  }
 public:
  calibration(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), computed(false), num_data(0){
    // Get parameters and show
    if(!pnh_.getParam("camera_name", camera_name)) camera_name = "camera";
    if(!pnh_.getParam("tag_name", tag_name)) tag_name = "tag_0";
    ROS_INFO("*************************\n \
\tcamera_name: %s \
\ttag_name: %s \
*************************", camera_name.c_str(), tag_name.c_str());
  matrix_M = Eigen::Matrix3f::Zero();
  }
  void printInfo(void){
    ROS_INFO("At least %d data points required, you have: %d data points now\n \
Press 'r' to record data, 'c' to compute", REQUIRED_DATA, num_data);
    char command; std::cin >> command;
    if(command=='c' and num_data<REQUIRED_DATA){
      ROS_WARN("No enough data, abort...");
    }else if(command=='r'){
      tf::StampedTransform stf;
      try{
        listener.waitForTransform("base_link", "ee_link", ros::Time(0), ros::Duration(0.5));
        listener.lookupTransform("base_link", "ee_link", ros::Time(0), stf);
        tf::Transform t(stf.getRotation(), stf.getOrigin());
        base2ee.push_back(t);
      } catch(tf::TransformException ex){
        ROS_WARN("%s", ex.what()); return;
      }
      try{
        listener.waitForTransform(camera_name+"link", tag_name, ros::Time(0), ros::Duration(0.5));
        listener.lookupTransform(camera_name+"link", tag_name, ros::Time(0), stf);
        tf::Transform t(stf.getRotation(), stf.getOrigin());
        cam2tag.push_back(t);
      } catch(tf::TransformException ex){
        ROS_WARN("%s", ex.what()); return;
      } ++num_data; ROS_INFO("Data logged.");
    } else if(command=='c'){
      compute_transform();
    } else {ROS_WARN("Invalid input, abort...");}
  }
  bool getStatus(void) {return computed;}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_eye_calibration_node");
  ros::NodeHandle nh, pnh("~");
  calibration Foo(nh, pnh);
  while(Foo.getStatus()) Foo.printInfo();
  return 0;
}
