#include "AdsLib.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  ROS_INFO_STREAM("running good");

  return 0;
}