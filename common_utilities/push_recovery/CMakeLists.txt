cmake_minimum_required(VERSION 2.8.3)
project(push_recovery)

find_package(catkin REQUIRED
  roscpp
  )

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} 	src/CPushRecovery.cpp
								src/CPushRecovery2DOF.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      )

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     
                           
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
