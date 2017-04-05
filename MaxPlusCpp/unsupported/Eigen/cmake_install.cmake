# Install script for directory: /home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/AdolcForward"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/AlignedVector3"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/ArpackSupport"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/AutoDiff"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/BVH"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/EulerAngles"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/FFT"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/IterativeSolvers"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/KroneckerProduct"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/LevenbergMarquardt"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/MatrixFunctions"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/MoreVectorization"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/MPRealSupport"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/NonLinearOptimization"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/NumericalDiff"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/OpenGLSupport"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/Polynomials"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/Skyline"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/SparseExtra"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/SpecialFunctions"
    "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/Splines"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/octaviovillarreal/Documents/build_dir/source_dir/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/octaviovillarreal/Documents/build_dir/unsupported/Eigen/CXX11/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

