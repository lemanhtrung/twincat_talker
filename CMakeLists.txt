cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)
project(twincat_talker)

find_package(catkin REQUIRED roscpp roslaunch)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs
)

add_subdirectory(dep/ADS)

include_directories(dep/ADS/AdsLib dep/ADS/AdsLibOOI ${catkin_INCLUDE_DIRS})

add_executable(twincat_talker src/main.cc)

target_link_libraries(twincat_talker
    adscxx
    ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

install(TARGETS twincat_talker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)