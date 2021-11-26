add_library(fastestlapc SHARED ./fastestlapc.cpp)

set_target_properties(fastestlapc PROPERTIES
    PUBLIC_HEADER ./fastestlapc.h
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(LFASTESTLAPC_ADDITIONAL_FLAGS -static-libstdc++ -static-libgcc)
endif()

target_link_libraries(fastestlapc LINK_PUBLIC lion::lion python ${LFASTESTLAPC_ADDITIONAL_FLAGS})

if ( NOT APPLE)
    target_link_options(fastestlapc PUBLIC -Wl,--no-as-needed -ldl)
endif()

configure_file(fastestlapc.h ${CMAKE_BINARY_DIR}/include/fastestlapc.h COPYONLY)

install(TARGETS fastestlapc)
