cmake_minimum_required(VERSION 2.8.3)
project(my_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  mav_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FourPWM.msg
  AngleRateThrottle.msg
  RotorPWM.msg
  VelCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES my_position_controller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime mav_msgs 
  DEPENDS Eigens
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/airsim_ros
  ${Eigen3_INCLUDE_DIRS}
#  ~/simulator/uasim_student_240204_r1_shipping/devel/include
)

add_library(my_position_controller
  src/library/my_position_controller.cpp
)
target_link_libraries(my_position_controller ${catkin_LIBRARIES})
add_dependencies(my_position_controller ${catkin_EXPORTED_TARGETS})

add_executable(my_position_controller_node src/nodes/my_position_controller_node.cpp)
add_dependencies(my_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_position_controller_node 
  my_position_controller  ${catkin_LIBRARIES})

install(TARGETS my_position_controller 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS my_position_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)