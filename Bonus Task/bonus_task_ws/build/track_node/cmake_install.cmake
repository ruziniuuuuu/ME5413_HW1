# Install script for directory: /home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/src/track_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/build/track_node/catkin_generated/installspace/track_node.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/track_node/cmake" TYPE FILE FILES
    "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/build/track_node/catkin_generated/installspace/track_nodeConfig.cmake"
    "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/build/track_node/catkin_generated/installspace/track_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/track_node" TYPE FILE FILES "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/src/track_node/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/track_node" TYPE PROGRAM FILES "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/build/track_node/catkin_generated/installspace/track_pub_p.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/track_node" TYPE PROGRAM FILES "/home/ruziniu/Projects/ME5413/Homework1_Perception/Bonus Task/bonus_task_ws/build/track_node/catkin_generated/installspace/track_sub_p.py")
endif()

