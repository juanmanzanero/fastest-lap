macro(new_test)
    get_filename_component(binary_name ${CMAKE_CURRENT_LIST_DIR} NAME)
    string(REPLACE " " "_" binary_name ${binary_name})
    
    set(BINARY ${binary_name}_test)
    
    file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp ../main.cpp)
    
    set(SOURCES ${TEST_SOURCES})
    
    add_executable(${BINARY} ${TEST_SOURCES})
    
    add_test(NAME ${BINARY} COMMAND ${BINARY})
    
    # Link libraries
    target_link_libraries(${BINARY} LINK_PRIVATE GTest::gtest lion::lion)
    
    # Copy required files
    add_custom_target(${BINARY}_link_data ALL
                      COMMAND ${CMAKE_COMMAND} -E create_symlink 
                      ${CMAKE_CURRENT_SOURCE_DIR}/data 
                      ${CMAKE_BINARY_DIR}/src/test/${binary_name}/data
                     )
    
    add_custom_target(${BINARY}_link_database ALL
                      COMMAND ${CMAKE_COMMAND} -E create_symlink 
                      ${CMAKE_SOURCE_DIR}/database 
                      ${CMAKE_BINARY_DIR}/src/test/${binary_name}/database
                     )

    if ( NOT APPLE)
        target_link_options(${BINARY} PUBLIC -Wl,--no-as-needed -ldl)
    endif()
	
endmacro()

add_subdirectory(./actuators)
add_subdirectory(./applications)
add_subdirectory(./chassis)
add_subdirectory(./tire)
add_subdirectory(./vehicles)


# Create a test if code coverage
if (CODE_COVERAGE)
	include(code-coverage)

	add_code_coverage_all_targets(EXCLUDE test/* googletest-src/*)

	set(BINARY coverage_test)

	file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false **/*.cpp main.cpp)

	set(SOURCES ${TEST_SOURCES})

	add_executable(${BINARY} ${TEST_SOURCES})

	target_link_libraries(${BINARY} PUBLIC gtest)

	set_target_properties(${BINARY} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

	target_code_coverage(${BINARY} AUTO ALL EXCLUDE test/* build/googletest-src/*) 
endif()
