find_package(Matlab)
if (Matlab_FOUND)
        execute_process(COMMAND 
             ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
             ${CMAKE_COMMAND} -E cmake_echo_color --green "-- Matlab found. MEX files will be compiled"
             )
	add_compile_options(-Wno-unused-variable)
	my_matlab_add_mex(NAME rear_axle SRC rear_axle_mex.cpp LINK_TO ${TINYXML_LIBRARY})
	include_directories(${TINYXML_INCLUDE_DIR})
else()
        execute_process(COMMAND 
             ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
             ${CMAKE_COMMAND} -E cmake_echo_color --orange "-- Matlab not found. MEX files will not be compiled"
             )
endif()
