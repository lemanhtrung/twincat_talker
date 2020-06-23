#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  ROS_INFO_STREAM("running good");

  static const AmsNetId remoteNetId{5, 57, 164, 100, 1, 1};
  static const char remoteIpV4[] = "10.180.30.220";

  AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

  AdsVariable<double> variable_to_write{route, "Bozzini_Masterarbeit.X_Pick"};
  AdsVariable<bool> control{route, "Bozzini_Masterarbeit.Send_Coordinates"};

  variable_to_write = 20.0;
  control = false;
  ROS_INFO_STREAM("changed variable on sps");

  return 0;
}
