cmake_minimum_required(VERSION 2.8.3)
project(signal_scope_headers)

find_package(PkgConfig)
pkg_check_modules (LCM lcm)
include_directories(${LCM_INCLUDE_DIRS})
link_directories(${LCM_LIBRARY_DIRS})

find_package(catkin REQUIRED
  roscpp
  )

catkin_package(
  INCLUDE_DIRS ${LCM_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp
)
