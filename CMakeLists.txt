cmake_minimum_required(VERSION 3.0.2)
project(light_painting)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_package()

install(
    DIRECTORY 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
