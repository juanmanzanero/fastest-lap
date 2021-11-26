if (${BUILD_GTEST})
include(ExternalProject)
ExternalProject_Add(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/thirdparty -DCMAKE_BUILD_TYPE=Release -DBUILD_GMOCK=OFF
    PREFIX ${THIRD_PARTY_DIR}/gtest
    SOURCE_DIR "${THIRD_PARTY_DIR}/gtest/source"
    BINARY_DIR "${THIRD_PARTY_DIR}/gtest/build"
    INSTALL_DIR ${THIRD_PARTY_DIR}/thirdparty
)
endif()
