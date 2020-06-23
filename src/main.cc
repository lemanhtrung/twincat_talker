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
    reference = tfMsgs[10];
    initialized = true;
    ROS_INFO_STREAM("System is initialized. Go on with new movement");
  }
  else
  {
    ROS_INFO_STREAM("Reference: " << reference.translation.x << "," << reference.translation.y << "," << reference.translation.z << std::endl;);
    geometry_msgs::Transform pickMsg = tfMsgs[5];
    geometry_msgs::Transform placeMsg = tfMsgs.back();
ROS_INFO_STREAM("Pick: " << pickMsg.translation.x << "," << pickMsg.translation.y << "," << pickMsg.translation.z << std::endl;);
ROS_INFO_STREAM("Place: " << placeMsg.translation.x << "," << placeMsg.translation.y << "," << placeMsg.translation.z << std::endl;);

    pickMsg.translation.x = pickMsg.translation.x - reference.translation.x;
    placeMsg.translation.x = placeMsg.translation.x - reference.translation.x;
    pickMsg.translation.y = pickMsg.translation.y - reference.translation.y;
    placeMsg.translation.y = placeMsg.translation.y - reference.translation.y;
    pickMsg.translation.z = pickMsg.translation.z - reference.translation.z;
    placeMsg.translation.z = placeMsg.translation.z - reference.translation.z;

    x_pick = -round(100*1000*pickMsg.translation.y)/100;
    y_pick = -round(100*1000*pickMsg.translation.x)/100;
    // z_pick = round(100*1000*pickMsg.translation.z)/100;
    z_pick = 0.0;

    x_place = -round(100*1000*placeMsg.translation.y)/100;
    y_place = -round(100*1000*placeMsg.translation.x)/100;
    // z_place = round(100*1000*placeMsg.translation.z)/100;
    z_place = 0.0;

    control = false;

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
