#include <ros/ros.h>

#include <algorithm>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");

  // load config from launch file
  Loader loader(Loader::getConfig(&nh_private));

  Lidar lidar;
  Odom odom;

  // input_bag_path: the bag should have two rostopics: odom and point_cloud
  /*
    lidar has a member std::vector<Scan> scans_ that push_backs each scan with 
    corresponding configs
  */
  std::string input_bag_path;
  ROS_INFO("Loading Pointcloud Data...");
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  } else if (!loader.loadPointcloudFromROSBag(
                 input_bag_path, Scan::getConfig(&nh_private), &lidar)) {
    ROS_FATAL("Error loading pointclouds from ROS bag.");
    exit(0);
  }

  /*
    odom, similar to lidar, has a member std::vector<Transform>
    T_o0_ot_ that push_backs each odom transform, which is vector
    consisted of a 3-d translation vector and a quaternion
  */
  bool transforms_from_csv;
  nh_private.param("transforms_from_csv", transforms_from_csv, false);
  std::string input_csv_path;
  ROS_INFO("Loading Transformation Data...                                ");
  if (transforms_from_csv) {
    if (!nh_private.getParam("input_csv_path", input_csv_path)) {
      ROS_FATAL("Could not find input_csv_path parameter, exiting");
      exit(EXIT_FAILURE);
    } else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
      ROS_FATAL("Error loading transforms from CSV.");
      exit(0);
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom)) {
    ROS_FATAL("Error loading transforms from ROS bag.");
    exit(0);
  }

  if (lidar.getNumberOfScans() == 0) {
    ROS_FATAL("No data loaded, exiting");
    exit(0);
  }

  /*
    lidar.setOdomOdomTransforms(odom) is used to associated each point in each scan 
    with linearly interpolated odom transform
  */
  ROS_INFO("Interpolating Transformation Data...                          ");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(Aligner::getConfig(&nh_private));

  // where optimization happens
  // the key concept is that, an added adjustment homogeneous transformation is introduced
  // to multiply odom transformation matrix at each timestamp when a cloud point is registered.
  // the odom transformation matrix for each cloud point registration is done via interpolation 
  // as mentioned above.
  // the optimization is to find the optimal adjustment homogeneous transformation matrix that
  // results in minimal distances between cloud points (clustered cloud points collectively form
  // semantic object shape) 
  aligner.lidarOdomTransform(&lidar, &odom);

  return 0;
}
