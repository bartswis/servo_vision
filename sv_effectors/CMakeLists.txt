cmake_minimum_required(VERSION 2.8.3)
project(sv_effectors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  std_msgs
  message_generation
)

add_service_files(
  FILES
  HeadAngles.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ve_head_test src/VE_head_test.cpp)
target_link_libraries(ve_head_test ${catkin_LIBRARIES})

 install(TARGETS ve_head_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

catkin_install_python(PROGRAMS
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

