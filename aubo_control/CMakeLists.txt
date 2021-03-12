cmake_minimum_required(VERSION 3.0.2)
project(aubo_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/aubo_control.launch)
#roslaunch_add_file_check(launch/teleop.launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)