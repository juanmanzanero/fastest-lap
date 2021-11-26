cmake_minimum_required(VERSION 3.10)

# set the project name
project(fastest-lap-third-party VERSION 1.0)

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(THIRD_PARTY_DIR ${CMAKE_BINARY_DIR})
set(PATCH_DIR ${CMAKE_SOURCE_DIR}/cmake/patch)

set(BUILD_GTEST ${BUILD_GTEST})
set(BUILD_LION ${BUILD_LION})
set(CMAKE_C_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER})
set(CMAKE_FORTRAN_COMPILER ${CMAKE_FORTRAN_COMPILER})
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})

## Configure doxygen
include(third-party/gtest)
include(third-party/lion)
