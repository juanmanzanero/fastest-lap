add_library(python INTERFACE)
set(Python3_FIND_STRATEGY LOCATION)
find_package(Python3 COMPONENTS Interpreter Development)

set(Python3_MATPLOTLIB FALSE)
if (Python3_FOUND)

    # Add python C++ interfaces
    target_link_libraries(python INTERFACE Python3::Python Python3::Module)
        
    # Add numpy C++ interfaces
    find_package(Python3 COMPONENTS NumPy)
    if(Python3_NumPy_FOUND)
      target_link_libraries(python INTERFACE Python3::NumPy)
      execute_process(COMMAND 
           ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
           ${CMAKE_COMMAND} -E cmake_echo_color --green "-- Numpy found"
           )
    else()
      execute_process(COMMAND 
           ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
           ${CMAKE_COMMAND} -E cmake_echo_color --orange "-- Numpy not found"
           )
      target_compile_definitions(python INTERFACE WITHOUT_NUMPY)
    endif()

    # Test if python has matplotlib
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import matplotlib"  RESULT_VARIABLE matplotlib_valid)
    if (${matplotlib_valid} MATCHES 0)
        execute_process(COMMAND 
             ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
             ${CMAKE_COMMAND} -E cmake_echo_color --green "-- Matplotlib found. Plotting tools will be available"
             )
        
	set(Python3_MATPLOTLIB TRUE)
    else()
        execute_process(COMMAND 
             ${CMAKE_COMMAND} -E env CLICOLOR_FORCE=1
             ${CMAKE_COMMAND} -E cmake_echo_color --orange "-- Matplotlib not found. Plotting tools will not be available"
             )
    endif()
endif()
