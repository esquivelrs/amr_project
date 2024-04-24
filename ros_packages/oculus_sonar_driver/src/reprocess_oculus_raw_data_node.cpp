// Copyright 2020-2023 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include "g3log_ros/ROSLogSink.h"
#include "g3log_ros/g3logger.h"
#include "nodelet/loader.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "reprocess_oculus_raw_data");

  libg3logger::G3Logger<ROSLogSink> log_worker(
      "reprocess_oculus_raw_data_node");

  nodelet::Loader nodelet(true);
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name + "/reprocess", "oculus_sonar/reprocess_raw_data",
               remap, nargv);
  // nodelet.load(nodelet_name + "/draw", "draw_sonar", remap, nargv);

  ros::spin();
  return 0;
}
