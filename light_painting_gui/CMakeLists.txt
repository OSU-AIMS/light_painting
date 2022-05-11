cmake_minimum_required(VERSION 3.0.2)
project(light_painting_gui)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)


################
## Catkin Pkg ##
################

catkin_package()
catkin_python_setup()


#############
## Install ##
#############

install(PROGRAMS scripts/light_painting_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
