# Install script for directory: /home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/src/image_publisher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/image_publisher" TYPE FILE FILES "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/devel/include/image_publisher/camera_cfgsConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/image_publisher" TYPE FILE FILES "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/devel/lib/python2.7/dist-packages/image_publisher/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/devel/lib/python2.7/dist-packages/image_publisher/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/image_publisher" TYPE DIRECTORY FILES "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/devel/lib/python2.7/dist-packages/image_publisher/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_publisher/srv" TYPE FILE FILES
    "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/src/image_publisher/srv/set_gain.srv"
    "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/src/image_publisher/srv/set_exposure.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/build/image_publisher/catkin_generated/installspace/image_publisher.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_publisher/cmake" TYPE FILE FILES
    "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/build/image_publisher/catkin_generated/installspace/image_publisherConfig.cmake"
    "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/build/image_publisher/catkin_generated/installspace/image_publisherConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_publisher" TYPE FILE FILES "/home/rouholla/myData/Robotics/projects/RaspiTrack/ros_ws/src/image_publisher/package.xml")
endif()

