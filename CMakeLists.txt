cmake_minimum_required(VERSION 3.0.2)
project(light_painting)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    message_generation

    roslib
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    descartes_utilities
    trajectory_msgs
    tf
    tf_conversions
    eigen_conversions
    tf2_eigen

    actionlib_msgs
    geometry_msgs
    std_msgs
)


#####################
## Custom Messages ##
#####################

add_action_files(
  DIRECTORY action
  FILES SimpleMoveRequest.action
)

add_message_files(
  FILES
  RGBState.msg
  BinaryState.msg
  GrayScale.msg
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs
    geometry_msgs
    std_msgs
)


###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    geometry_msgs
    message_runtime
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_single_move_action src/descartes_motion_action_server/run_server.cpp)

add_dependencies(
  ${PROJECT_NAME}_single_move_action ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}_single_move_action
    ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_single_move_action
    PROPERTIES OUTPUT_NAME single_move_action
    PREFIX "")


#############
## Install ##
#############

install(
    DIRECTORY launch scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_single_move_action
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
