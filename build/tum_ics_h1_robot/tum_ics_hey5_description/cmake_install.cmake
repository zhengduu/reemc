# Install script for directory: /home/duzheng/ros/workspaces/duzheng_test_ws/src/tum_ics_h1_robot/tum_ics_hey5_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/duzheng/ros/workspaces/duzheng_test_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/duzheng/ros/workspaces/duzheng_test_ws/build/tum_ics_h1_robot/tum_ics_hey5_description/catkin_generated/installspace/tum_ics_hey5_description.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_hey5_description/cmake" TYPE FILE FILES
    "/home/duzheng/ros/workspaces/duzheng_test_ws/build/tum_ics_h1_robot/tum_ics_hey5_description/catkin_generated/installspace/tum_ics_hey5_descriptionConfig.cmake"
    "/home/duzheng/ros/workspaces/duzheng_test_ws/build/tum_ics_h1_robot/tum_ics_hey5_description/catkin_generated/installspace/tum_ics_hey5_descriptionConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_hey5_description" TYPE FILE FILES "/home/duzheng/ros/workspaces/duzheng_test_ws/src/tum_ics_h1_robot/tum_ics_hey5_description/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_hey5_description/meshes" TYPE DIRECTORY FILES "/home/duzheng/ros/workspaces/duzheng_test_ws/src/tum_ics_h1_robot/tum_ics_hey5_description/meshes/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_hey5_description/urdf" TYPE DIRECTORY FILES "/home/duzheng/ros/workspaces/duzheng_test_ws/src/tum_ics_h1_robot/tum_ics_hey5_description/urdf/")
endif()

