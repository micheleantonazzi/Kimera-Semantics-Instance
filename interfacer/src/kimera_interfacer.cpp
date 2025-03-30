//  catkin build kimera_interfacer --no-deps

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
//#include "kimera_interfacer/SyncSemantic.h"
#include "kimera_semantics_ros/depth_map_to_pointcloud.h"
//#include "kimera_semantics_ros/depth_map_to_pointcloud.h"
#include "kimera_semantics_ros/rosbag_data_provider.h"
#include "kimera_semantics_ros/semantic_tsdf_server.h"

#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>
#include <tf2_ros/transform_listener.h>
#include <minkindr_conversions/kindr_tf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace cv;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "interfacer");
  std::cout << "Started the Interfacer Node for kimera" << std::endl;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  return EXIT_SUCCESS;
}
