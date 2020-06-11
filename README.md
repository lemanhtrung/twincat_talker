# ROS interface for Beckhoff TwinCAT PLC :robot: :rotating_light:

## :warning: STILL UNDER DEVELOPMENT :warning:

## Purpose

This ROS package enables to overwrite variables of a PLC project running on a Beckhoff TwinCAT PLC from ROS topics. \
For further information:

- [Beckhoff ADS library](https://infosys.beckhoff.com/content/1033/tc3_ads_intro/index.html)
- [Beckhoff ADS Github](https://github.com/Beckhoff/ADS)

## Installation

Clone the project into your catkin_ws/src \
`git clone https://github.com/hslu-c2a/tctalker.git` \
Build the project \
`catkin_make` \
Start a roscore and in a separate shell \
`source devel/bash.setup && rosrun twincat_talker tctalker`
