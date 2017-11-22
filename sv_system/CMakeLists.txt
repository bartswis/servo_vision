cmake_minimum_required(VERSION 2.8.3)
project(sv_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  visualization_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)

FILE(GLOB LIB_SRC_FILES "src/lib/*.cpp")

add_library(sv_system_lib ${LIB_SRC_FILES})

add_executable(control_system_test src/CS_test.cpp)
target_link_libraries(control_system_test ${catkin_LIBRARIES} sv_system_lib)

 install(TARGETS control_system_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
