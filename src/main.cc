#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tfMessage.h>
#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"

static const AmsNetId remoteNetId{ 5, 57, 164, 100, 1, 1 };
static const char remoteIpV4[] = "10.180.30.220";
AdsDevice route{ remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3 };

AdsVariable<double> x_pick{ route, "Bozzini_Masterarbeit.X_Pick" };
AdsVariable<double> y_pick{ route, "Bozzini_Masterarbeit.Y_Pick" };
AdsVariable<double> z_pick{ route, "Bozzini_Masterarbeit.Z_Pick" };
AdsVariable<double> c_pick{ route, "Bozzini_Masterarbeit.C_Pick" };

AdsVariable<double> x_place{ route, "Bozzini_Masterarbeit.X_Place" };
AdsVariable<double> y_place{ route, "Bozzini_Masterarbeit.Y_Place" };
AdsVariable<double> z_place{ route, "Bozzini_Masterarbeit.Z_Place" };
AdsVariable<double> c_place{ route, "Bozzini_Masterarbeit.C_Place" };

AdsVariable<bool> control{ route, "Bozzini_Masterarbeit.Send_Coordinates" };

std::vector<geometry_msgs::Transform> tfMsgs;

std_msgs::Bool myBool;

bool initialized = false;
geometry_msgs::Transform reference;

void sendDataToSps()
{
  if (!initialized)
  {
    reference = tfMsgs[0];
    initialized = true;
    ROS_INFO_STREAM("System is initialized. Go on with new movement");
  }
  else
  {
    geometry_msgs::Transform pickMsg = tfMsgs[5];
    geometry_msgs::Transform placeMsg = tfMsgs[-5];

    pickMsg.translation.x = pickMsg.translation.x - reference.translation.x;
    placeMsg.translation.x = placeMsg.translation.x - reference.translation.x;
    pickMsg.translation.y = pickMsg.translation.y - reference.translation.y;
    placeMsg.translation.y = placeMsg.translation.y - reference.translation.y;
    pickMsg.translation.z = pickMsg.translation.z - reference.translation.z;
    placeMsg.translation.z = placeMsg.translation.z - reference.translation.z;

    x_pick = pickMsg.translation.x;
    y_pick = pickMsg.translation.y;
    z_pick = pickMsg.translation.z;

    x_pick = placeMsg.translation.x;
    y_pick = placeMsg.translation.y;
    z_pick = placeMsg.translation.z;

    control = true;

    ROS_INFO_STREAM("Data sent successfully to SPS!");
  }
}

int main(int argc, char *argv[])
{
  tfMsgs.clear();
  myBool.data = false;

  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  ROS_INFO_STREAM("running good");

  boost::function<void(const tf::tfMessage::ConstPtr &)> tfCallback = [&](const tf::tfMessage::ConstPtr tfMsg) {
    if (myBool.data == true)
    {
      tfMsgs.emplace_back(tfMsg->transforms.begin()->transform);
    }
  };

  boost::function<void(const std_msgs::Bool::ConstPtr &)> boolCallback = [&](const std_msgs::Bool::ConstPtr boolMsg) {
    if ((myBool.data == true) && (boolMsg->data == false))
    {
      // SEND DATA TO SPS AND CLEAR VECTOR
      myBool = *boolMsg.get();
      sendDataToSps();
      tfMsgs.clear();
    }
    myBool = *boolMsg.get();
  };

  ros::Subscriber tfSub = n.subscribe("tf", 100, tfCallback);
  ros::Subscriber boolSub = n.subscribe("keyboard/bool", 100, boolCallback);

  ros::spin();

  return 0;
}
