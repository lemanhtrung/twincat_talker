#include <geometry_msgs/Transform.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tfMessage.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
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

int mode;

bool initialized = false;
geometry_msgs::Transform reference;

Eigen::Matrix3d quat2Rot(const geometry_msgs::Quaternion &quat)
{
  Eigen::Matrix3d rotM;
  double qw = quat.w;
  double qx = quat.x;
  double qy = quat.y;
  double qz = quat.z;

  rotM(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
  rotM(0, 1) = 2 * qx * qy - 2 * qz * qw;
  rotM(0, 2) = 2 * qx * qz + 2 * qy * qw;

  rotM(1, 0) = 2 * qx * qy + 2 * qz * qw;
  rotM(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
  rotM(1, 2) = 2 * qy * qz - 2 * qx * qw;

  rotM(2, 0) = 2 * qx * qz - 2 * qy * qw;
  rotM(2, 1) = 2 * qy * qz + 2 * qx * qw;
  rotM(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

  return rotM;
}

Eigen::Vector3d rot2Euler(const Eigen::Matrix3d &rot)
{
  Eigen::Vector3d euler;  // [psi, theta, phi]

  if ((rot(2, 0) != 1) | (rot(2, 0) != -1))
  {
    euler(1) = -asin(rot(2, 0));
    euler(0) = atan2(rot(2, 1) / cos(euler(1)), rot(2, 2) / cos(euler(1)));
    euler(2) = atan2(rot(1, 0) / cos(euler(1)), rot(0, 0) / cos(euler(1)));
  }
  else
  {
    euler(2) = 0;
    if (rot(2, 0) == -1)
    {
      euler(1) = M_PI / 2;
      euler(0) = euler(2) + atan2(rot(0, 1), rot(0, 2));
    }
    else
    {
      euler(1) = -M_PI / 2;
      euler(0) = -euler(1) + atan2(-rot(0, 1), -rot(0, 2));
    }
  }

  // convert do degrees
  euler(1) = euler(1) * 180.0 / M_PI;
  euler(0) = euler(0) * 180.0 / M_PI;
  euler(2) = euler(2) * 180.0 / M_PI;

  return euler;
}

void sendToSps()
{
  ROS_INFO_STREAM("Reference: " << reference.translation.x << "," << reference.translation.y << ","
                                << reference.translation.z << std::endl;);
  geometry_msgs::Transform pickMsg = tfMsgs[5];
  geometry_msgs::Transform placeMsg = tfMsgs.back();
  ROS_INFO_STREAM("Pick: " << pickMsg.translation.x << "," << pickMsg.translation.y << "," << pickMsg.translation.z
                           << std::endl;);
  ROS_INFO_STREAM("Place: " << placeMsg.translation.x << "," << placeMsg.translation.y << "," << placeMsg.translation.z
                            << std::endl;);

  pickMsg.translation.x = pickMsg.translation.x - reference.translation.x;
  placeMsg.translation.x = placeMsg.translation.x - reference.translation.x;
  pickMsg.translation.y = pickMsg.translation.y - reference.translation.y;
  placeMsg.translation.y = placeMsg.translation.y - reference.translation.y;
  pickMsg.translation.z = pickMsg.translation.z - reference.translation.z;
  placeMsg.translation.z = placeMsg.translation.z - reference.translation.z;

  x_pick = -round(100 * 1000 * pickMsg.translation.y) / 100;
  y_pick = -round(100 * 1000 * pickMsg.translation.x) / 100;
  // z_pick = round(100*1000*pickMsg.translation.z)/100;
  z_pick = 0.0;

  x_place = -round(100 * 1000 * placeMsg.translation.y) / 100;
  y_place = -round(100 * 1000 * placeMsg.translation.x) / 100;
  // z_place = round(100*1000*placeMsg.translation.z)/100;
  z_place = 0.0;

  control = false;

  ROS_INFO_STREAM("Data sent successfully to SPS!");
}

void writeToFile()
{
  Eigen::Matrix3d refRot = quat2Rot(reference.rotation);

  std::string file_name = "/run/user/1000/gvfs/smb-share:server=fs01e.eee.intern,share=data$/30 "
                          "CCMS/40_Projekte/140_MTPascalDemo/backtracking.txt";
  std::ofstream file;
  file.open(file_name);
  file << "# RoboBahn[i]={{x,y,z,-180+a,0+b,-180+c},{sfree,efree,wfree}}" << std::endl;
  ROS_INFO_STREAM(tfMsgs.size() << " points to send."); 
  for (int idx = 0; idx < tfMsgs.size(); ++idx)
  {
    geometry_msgs::Transform tfMsg = tfMsgs[idx];
    Eigen::Vector3d transl;  // [x,y,z]
    Eigen::Vector3d euler;   // [a,b,c]
    // calculate translation
    transl(0) = tfMsg.translation.x - reference.translation.x;
    transl(1) = tfMsg.translation.y - reference.translation.y;
    transl(2) = tfMsg.translation.z - reference.translation.z;

    transl(0) = round(100 * 1000 * transl(0)) / 100;
    transl(1) = round(100 * 1000 * transl(1)) / 100;
    transl(2) = round(100 * 1000 * transl(2)) / 100;

    // calculate orientation
    Eigen::Matrix3d msgRot = quat2Rot(tfMsg.rotation);
    Eigen::Matrix3d diffRot = refRot.transpose() * msgRot;
    euler = rot2Euler(diffRot);

    // correction of angles
    euler(0) = 0; // + euler(0);
    euler(1) = -180; // + euler(1);
    euler(2) = 0 - euler(2);

    // write to file
    file << "RoboBahn[" << idx << "]={{" << -transl(1) << "," << -transl(0) << "," << -transl(2) << "," << euler(0) << ","
         << euler(1) << "," << euler(2) << "},{sfree,efree,wfree}}" << std::endl;
  }

  file.close();

  ROS_INFO_STREAM("Data written so file!");
}

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
    if (mode == 1)
    {
      sendToSps();
    }
    else if (mode == 2)
    {
      writeToFile();
    }
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
  std::cout << "Please enter mode (1: pick and place; 2: backtracking): ";
  std::cin >> mode;

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

  ros::Subscriber tfSub = n.subscribe("tf", 1, tfCallback);
  ros::Subscriber boolSub = n.subscribe("keyboard/bool", 1, boolCallback);

  ros::spin();

  return 0;
}
