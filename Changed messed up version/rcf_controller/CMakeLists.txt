cmake_minimum_required(VERSION 2.8.3)
project(rcf_controller)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  pluginlib
  dls_controller
  central_pattern_generator
  kinematic_adjustment
  push_recovery
  terrain_estimator
  hyq2max_ik
  hyq_ik
  math_utils
  kalman_state_estimator)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib
  DEPENDS system_lib
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories("$ENV{PRONTO_ROOT}/build/include")

# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/support)

## Declare a cpp library
add_library(${PROJECT_NAME} src/RCFController.cpp
                            src/RCFConsole.cpp
                            src/support/HyQ2MaxWorkspaceCheck.cpp
                            src/support/CPIDManager.cpp
                            src/support/CTerrainBasedAdjustment.cpp
                            src/support/CTrunkController.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${dwl_LIBRARIES}
                                      ${GENCODELIB}
                                      iitgenhyq2max
                                      iitgenhyq
                                      readline)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     
                           
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

