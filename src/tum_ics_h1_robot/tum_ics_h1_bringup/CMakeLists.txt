cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_h1_bringup)

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED)

catkin_package()

# This function is controlled with the global variable -DDEB_BUILD (true/false)
include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
ics_ros_debgen()

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

