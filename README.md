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

## Usage

Change the AmsNetId in the main.cc to the one observed on your PLC \
`static const AmsNetId remoteNetId{x, x, x, x, x, x};` \
Change the remoteIpV4 in the main.cc to the IP of the PLC \
`static const remoteIpV4[] = "x.x.x.x";` \
Setup the ADS device with the create variables above and the correct port at the end \
`AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};`
\
Add an AdsVariable you want to change with the name NAME \
`AdsVariable<double> variable_to_write{route, "NAME"}` \
Now you can overwrite the variable with the desired value \
`variable_to_write = 12.0;`

## Data type conversion

| Data type | Beckhoff TC3 | cpp       |
| --------- | ------------ | --------- |
| boolean   | BOOL         | `bool`    |
| byte      | BYTE         | `byte`    |
| short     | INT          | `int32_t` |
| long      | LINT         | `int64_t` |
| float     | REAL         | `float`   |
| double    | LREAL        | `double`  |
