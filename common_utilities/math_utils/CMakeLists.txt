cmake_minimum_required(VERSION 2.8.3)
project(math_utils)

find_package(catkin REQUIRED
  roscpp
  )

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS roscpp
)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

