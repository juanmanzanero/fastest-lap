cmake_minimum_required(VERSION 3.10)

# set the project name
project(fastest-lap VERSION 1.0)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/include)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

option(CHECK_BOUNDS "Enable bounds check at runtime" OFF)
option(ENABLE_TESTS "Enable testing" ON)
option(PYTHON_API_ABSOLUTE_PATH "Use absolute paths to look for libraries in the python API" ON)

message("")
message("	Configure third party libraries ")
message("	=============================== ")
message("")
include(get-third-party)
message("")
execute_process(COMMAND 
     ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
     ${CMAKE_COMMAND} -E cmake_echo_color --green "-> [DONE]"
     )

message("")

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

# default build type is Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("	Configure Fastest-lap")
message("	=====================")
message("")
include_directories(./)

# Google tests
enable_testing()
list(APPEND CMAKE_CTEST_ARGUMENTS "--verbose")

# Compile the subdirectories
include(compilerflags)
include(matlabutils)

add_subdirectory(./src)

if ( ${PYTHON_API_ABSOLUTE_PATH} )
	set(libdir_python ${CMAKE_BINARY_DIR}/lib)
else()
	file(RELATIVE_PATH libdir_python ${CMAKE_SOURCE_DIR}/examples/python/ ${CMAKE_BINARY_DIR}/lib)
endif()

if ( MSYS AND ${CMAKE_BUILD_TYPE} STREQUAL "Debug" )
    set(CONSTRUCT_LIBFASTESTLAPC OFF)
else()
    set(CONSTRUCT_LIBFASTESTLAPC ON)
endif()

if (${CONSTRUCT_LIBFASTESTLAPC})
    configure_file(${CMAKE_SOURCE_DIR}/src/main/python/fastest_lap.py ${CMAKE_SOURCE_DIR}/examples/python/fastest_lap.py)
    file(GENERATE OUTPUT ${CMAKE_SOURCE_DIR}/examples/python/fastest_lap.py INPUT ${CMAKE_SOURCE_DIR}/examples/python/fastest_lap.py)
endif()

install(DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty/lib/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")



