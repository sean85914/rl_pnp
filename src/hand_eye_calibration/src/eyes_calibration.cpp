#include <ros/ros.h>
#include <tf/transform_listener.h>

// Four camera calibration using one scen with apriltag
bool status = true;
const std::string TAG_FRAME = "tag_1";
tf::Transform transform_vector[4];

void print_transform(tf::Transform t){
  ROS_INFO("\nTranslation: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]", 
           t.getOrigin().getX(), t.getOrigin().getY(), t.getOrigin().getZ(),
           t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ(), t.getRotation().getW());
}

tf::Transform get_transform(int id){
  static tf::TransformListener listener;
  ROS_INFO("Get transformation from camera %d to tag", id);
  std::string camera_frame = "camera"+std::to_string(id)+"/zed_camera_center";
  tf::StampedTransform stf;
  try{
    listener.waitForTransform(camera_frame, TAG_FRAME, ros::Time(0), ros::Duration(30.0));
    listener.lookupTransform(camera_frame, TAG_FRAME, ros::Time(0), stf);
  } catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    status = false; ROS_ERROR("Cannot get transformation between tags!");
  }
  tf::Transform result(stf.getRotation(), stf.getOrigin());
  print_transform(result); printf("return \n");
  return result;
}

/*void process(void){
  for(int i=0; i<4; ++i) {transform_vector[i] = get_transform(i+1); printf("i: %d\n", i);}
  ROS_INFO("Processing data...");
  for(int i=1; i<4; ++i){
    ROS_INFO("Camera 1 to Camera %d", i+1);
    print_transform(transform_vector[0]*transform_vector[i].inverse());
  } status = false;
}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eyes_calibration_node");
  for(int i=0; i<4; ++i) {
    transform_vector[i] = get_transform(i+1);
    if(!status) ros::shutdown();
  }
  for(int i=1; i<4; ++i){
    ROS_INFO("Camera 1 to Camera %d", i+1);
    print_transform(transform_vector[0]*transform_vector[i].inverse());
  }
  return 0;
}
