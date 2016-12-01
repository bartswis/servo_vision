cmake_minimum_required(VERSION 2.8.3)
project(sv_receptors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  cv_bridge
  image_transport
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vr_camera src/VRCamera.cpp)
target_link_libraries(vr_camera ${catkin_LIBRARIES})

 install(TARGETS vr_camera
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
