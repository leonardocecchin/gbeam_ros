cmake_minimum_required(VERSION 3.0.2)
project(gbeam_library)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
	message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ReachabilityGraph.msg
 Vertex.msg
 GraphEdge.msg
 PolyArea.msg
 FreePolygon.msg
 FreePolygonStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  setMappingStatus.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 sensor_msgs
 nav_msgs
 geometry_msgs
 std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
INCLUDE_DIRS include
LIBRARIES graph_fcn polytope_fcn exploration_fcn
CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(graph_fcn
  src/graph_fcn.cpp
)
add_library(polytope_fcn
src/polytope_fcn.cpp
)
add_library(exploration_fcn
src/exploration_fcn.cpp
)

add_dependencies(graph_fcn
${catkin_EXPORTED_TARGETS}
gbeam_library_generate_messages_cpp
)
add_dependencies(polytope_fcn
${catkin_EXPORTED_TARGETS}
gbeam_library_generate_messages_cpp
)
add_dependencies(exploration_fcn
${catkin_EXPORTED_TARGETS}
gbeam_library_generate_messages_cpp
)

target_link_libraries(graph_fcn
${catkin_LIBRARIES}
)
target_link_libraries(polytope_fcn
${catkin_LIBRARIES}
)
target_link_libraries(exploration_fcn
${catkin_LIBRARIES}
)


# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS graph_fcn polytope_fcn exploration_fcn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gbeam_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
