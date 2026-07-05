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
    
    set_target_properties(fastestlapc PROPERTIES 
	CXX_VISIBILITY_PRESET hidden
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
    
    set_target_properties(fastestlapc PROPERTIES
        PUBLIC_HEADER ./fastestlapc.h
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    
    # NOTE: do NOT statically link libstdc++/libgcc into this SHARED library.
    # It is loaded into processes (e.g. the Python API) that already load the
    # dynamic libstdc++.so.6 transitively (libgfortran, the scientific stack).
    # Two libstdc++ runtimes in one process gives std::locale/std::regex facets
    # with mismatched vtables -> virtual dispatch lands in the wrong method and
    # segfaults (observed: SIGSEGV in std::regex compilation via trim() during
    # optimal_laptime setup on aarch64). Use the single system libstdc++.
    set(LFASTESTLAPC_ADDITIONAL_FLAGS)

    target_link_libraries(fastestlapc LINK_PUBLIC lion::lion ${LFASTESTLAPC_ADDITIONAL_FLAGS})
    
    if ( NOT APPLE)
        target_link_options(fastestlapc PUBLIC -Wl,--no-as-needed -ldl)
    endif()
    
    configure_file(fastestlapc.h ${CMAKE_BINARY_DIR}/include/fastestlapc.h COPYONLY)
    
    install(TARGETS fastestlapc)
endif()
