#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  // load parameters

  std::string ip_address;
  n.getParam("/twincat_talker/ip_address", ip_address);
  char remoteIpV4[ip_address.size() + 1];
  strcpy(remoteIpV4, ip_address.c_str());
  ROS_INFO_STREAM("ip address of target: " << ip_address);

  std::vector<int> remote_net_id;
  n.getParam("/twincat_talker/remoteNetId", remote_net_id);
  static const AmsNetId remoteNetId{remote_net_id.at(0), remote_net_id.at(1),
                                    remote_net_id.at(2), remote_net_id.at(3),
                                    remote_net_id.at(4), remote_net_id.at(5)};
  ROS_INFO_STREAM("remote net id of target: "
                  << remote_net_id.at(0) << "," << remote_net_id.at(1) << ","
                  << remote_net_id.at(2) << "," << remote_net_id.at(3) << ","
                  << remote_net_id.at(4) << "," << remote_net_id.at(5));

  std::string plc_variable;
  n.getParam("/twincat_talker/variable", plc_variable);
  ROS_INFO_STREAM("variable to write on plc: " << plc_variable);

  ROS_INFO_STREAM("running good");

  AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

  AdsVariable<double> variable_to_write{route, plc_variable};

  ros::Rate loop_rate(0.1);

  while (ros::ok()) {
    variable_to_write = 12.0;
    ROS_INFO_STREAM("changed variable on sps");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
