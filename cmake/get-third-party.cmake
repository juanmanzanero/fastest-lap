# Google tests 
find_package(GTest PATHS ${CMAKE_BINARY_DIR})

if ( NOT ${GTest_FOUND})
    set(BUILD_GTEST YES)
endif()

# Lion
set(BUILD_LION YES)



#######################################################################




##### BUILD ALL REQUIRED THIRD PARTY LIBRARIES ##########
message(STATUS "Compilation of the required third party libraries")
configure_file(cmake/third-party/CMakeLists.txt ${CMAKE_BINARY_DIR}/thirdparty/CMakeLists.txt)

execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
)
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
)
message("")
message(STATUS "Configuration of fastest-lap")
#######################################################################


find_package(GTest PATHS ${CMAKE_BINARY_DIR}/thirdparty REQUIRED)
find_package(lion PATHS ${CMAKE_BINARY_DIR}/thirdparty/lib/cmake/lion HINTS ${CMAKE_BINARY_DIR}/thirdparty/lib/cmake/lion NO_DEFAULT_PATH REQUIRED)
