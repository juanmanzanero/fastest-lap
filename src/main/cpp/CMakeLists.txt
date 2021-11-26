add_executable(fastest-lap ./main.cpp)

set_target_properties(fastest-lap PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
   )

target_link_libraries(fastest-lap LINK_PUBLIC lion::lion python)

if ( NOT APPLE)
    target_link_options(fastest-lap PUBLIC -Wl,--no-as-needed -ldl -pthread)
endif()

install(TARGETS fastest-lap)
