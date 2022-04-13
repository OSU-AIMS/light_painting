cmake_minimum_required(VERSION 3.0.2)
project(light_painting)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    std_msgs
    message_generation
)


####################
## Custom Message ##
####################

add_message_files(
  FILES
  RGBState.msg
  BinaryState.msg
  GrayScale.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS message_runtime
)


#############
## Install ##
#############

install(
    DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
