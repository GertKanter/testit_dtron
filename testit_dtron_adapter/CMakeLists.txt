cmake_minimum_required(VERSION 2.8.3)
project(testit_dtron_adapter)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	genmsg
	tf
	move_base_msgs
	actionlib
)

SET(${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${CATKIN_PACKAGE_LIB_DESTINATION})
SET(Spread_INCLUDE_DIRS "/catkin_ws/spread/include")
SET(PROTOBUF_INCLUDE_DIRS "/catkin_ws/protobuf/include")
SET(topological_navigation_INCLUDE_DIRS "/catkin_ws/devel/include")

catkin_package(
  INCLUDE_DIRS include
  #CATKIN_DEPENDS other_catkin_pkg
  DEPENDS Spread
)

include_directories(
${topological_navigation_INCLUDE_DIRS}
include
 ${catkin_INCLUDE_DIRS}
 ${PROTOBUF_INCLUDE_DIRS}
 ${Spread_INCLUDE_DIRS}
)

add_library(xtaproto_lib
  src/xtaproto.pb.cc
)

add_executable(test_adapter src/test_adapter.cpp)

find_library(protobuf protobuf PATHS ${PROJECT_SOURCE_DIR}/protobuf/lib NO_CMAKE_SYSTEM_PATH)
get_filename_component(protobuf_REALPATH ${protobuf} REALPATH)

find_library(spread spread PATHS ${PROJECT_SOUCE_DIR}/spread/lib NO_CMAKE_SYSTEM_PATH)
get_filename_component(spread_REALPATH ${spread} REALPATH)

find_library(topological_navigation topological_navigation PATHS /catkin_ws/devel/lib NO_CMAKE_SYSTEM_PATH)
get_filename_component(topological_navigation_REALPATH ${topological_navigation} REALPATH)

target_link_libraries(test_adapter
   ${catkin_LIBRARIES}
   ${spread_REALPATH}
   ${protobuf_REALPATH}
   xtaproto_lib
   pthread
 )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_spread_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
