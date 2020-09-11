# ROS interface for Beckhoff TwinCAT PLC :robot: :rotating_light:

## :warning: STILL UNDER DEVELOPMENT :warning:

## Purpose

This ROS package enables to overwrite variables of a PLC project running on a Beckhoff TwinCAT PLC from ROS topics. \
For further information:

- [Beckhoff ADS library](https://infosys.beckhoff.com/content/1033/tc3_ads_intro/index.html)
- [Beckhoff ADS Github](https://github.com/Beckhoff/ADS)

## Installation

Clone the project into your `catkin_ws/src` \
`git clone https://github.com/hslu-c2a/twincat_talker.git` \
Build the project \
`catkin_make` \
Start a roscore and in a separate shell \
`source devel/bash.setup && rosrun twincat_talker twincat_talker`

## Usage

Set the correct parameters the [parameter file](config/plc.yaml) \
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
