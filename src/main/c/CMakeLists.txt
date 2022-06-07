if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if ( MSYS AND ${CMAKE_BUILD_TYPE} STREQUAL "Debug" )
    set(CONSTRUCT_LIBFASTESTLAPC OFF)
else()
    set(CONSTRUCT_LIBFASTESTLAPC ON)
endif()

if ( ${CONSTRUCT_LIBFASTESTLAPC} )
    add_library(fastestlapc SHARED ./fastestlapc.cpp)
    
    set_target_properties(fastestlapc PROPERTIES CXX_VISIBILITY_PRESET hidden)
    
    set_target_properties(fastestlapc PROPERTIES
        PUBLIC_HEADER ./fastestlapc.h
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    
    if (NOT MSYS)
    	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    		set(LFASTESTLAPC_ADDITIONAL_FLAGS -static-libstdc++ -static-libgcc)
    	endif()
    endif()
    
    target_link_libraries(fastestlapc LINK_PUBLIC lion::lion ${LFASTESTLAPC_ADDITIONAL_FLAGS})
    
    if ( NOT APPLE)
        target_link_options(fastestlapc PUBLIC -Wl,--no-as-needed -ldl)
    endif()
    
    configure_file(fastestlapc.h ${CMAKE_BINARY_DIR}/include/fastestlapc.h COPYONLY)
    
    install(TARGETS fastestlapc)
endif()
