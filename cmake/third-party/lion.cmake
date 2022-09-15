if (${BUILD_LION})

set(lion_version f313fcc5b207991447692d3ee4391a6e93233ea1)

include(ExternalProject)
ExternalProject_Add(lion
    GIT_REPOSITORY https://github.com/juanmanzanero/lion-cpp.git
    GIT_TAG ${lion_version}
    CMAKE_ARGS -DENABLE_TEST=OFF -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/thirdparty -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    PREFIX ${THIRD_PARTY_DIR}/lion
    SOURCE_DIR "${THIRD_PARTY_DIR}/lion/source"
    BINARY_DIR "${THIRD_PARTY_DIR}/lion/build"
    INSTALL_DIR ${THIRD_PARTY_DIR}
)
endif()
