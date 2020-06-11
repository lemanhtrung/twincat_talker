#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  ROS_INFO_STREAM("running good");

  static const AmsNetId remoteNetId{10, 180, 30, 120, 1, 1};
  static const char remoteIpV4[] = "ads-server";

  AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

  AdsVariable<float_t> variable_to_write{route, "MAIN.myVar"};

  variable_to_write = 4.0f;

  return 0;
}