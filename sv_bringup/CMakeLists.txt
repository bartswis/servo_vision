cmake_minimum_required(VERSION 2.8.3)
project(sv_bringup)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

