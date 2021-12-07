cmake_minimum_required(VERSION 3.10)

# set the project name
project(fastest-lap VERSION 1.0)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/include)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(get-third-party)

## Configure doxygen
include(Doxygen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS ON)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")


include_directories(./)

# Google tests
enable_testing()
list(APPEND CMAKE_CTEST_ARGUMENTS "--verbose")

# Compile the subdirectories
include(compilerflags)
include(matlabutils)
add_subdirectory(./src)

file(GENERATE OUTPUT ${CMAKE_SOURCE_DIR}/examples/python/fastest_lap.py INPUT ${CMAKE_SOURCE_DIR}/src/main/python/fastest_lap.py)

install(DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty/lib/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")



