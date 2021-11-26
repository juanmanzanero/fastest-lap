add_compile_options(-Wfatal-errors -Werror -Wall)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O0)
endif()

if(CHECK_BOUNDS)
    message(STATUS "Check bounds ON")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
 	add_compile_options(-fsanitize=address)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options(-D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC)
    endif()
endif()


#link_directories(${CMAKE_BINARY_DIR}/lib)
