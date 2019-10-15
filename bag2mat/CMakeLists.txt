cmake_minimum_required(VERSION 2.8.3)

# Project name
project(bag2mat)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS rosbag roslib)

# Describe catkin Project
catkin_package(
        CATKIN_DEPENDS 
#        INCLUDE_DIRS src
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")

# Include our header files
#include_directories(
#        src
#        ${catkin_INCLUDE_DIRS}
#)

# Set link libraries used by all binaries
list(APPEND parser_libraries
        ${catkin_LIBRARIES}
)


##################################################
# Make binary for the offline reader
##################################################
#add_executable(bag2mat
#        src/bag2mat.cpp
#)
#target_link_libraries(bag2mat ${parser_libraries})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  scripts/bag2matpy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
