if (${BUILD_LION})

set(lion_version e1e4ac06bcbf95e0aeaddcdebabc244e4df08a88)

# logger-cpp is pulled in by lion pinned to a moving 'main' branch. Upstream has
# since (a) renamed logger.hpp -> logger.h and (b) moved the global `out`/`err`
# loggers into static data members, both of which break this (pinned) lion and
# fastest-lap, which include "logger.hpp" and call `out(...)` unqualified. Pin
# logger-cpp to the last commit before those changes (f2819e6, 2021-10-09), which
# still ships logger.hpp and the global loggers. Applied as a patch here so the
# fix lives in this repo without forking lion-cpp.
set(loggercpp_version f2819e68b4b1edb345333c72c9dba744e22357a7)

include(ExternalProject)
ExternalProject_Add(lion
    GIT_REPOSITORY https://github.com/juanmanzanero/lion-cpp.git
    GIT_TAG ${lion_version}
    PATCH_COMMAND sed -i "s/GIT_TAG main/GIT_TAG ${loggercpp_version}/" cmake/third-party/loggercpp.cmake
    CMAKE_ARGS -DENABLE_TEST=OFF -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/thirdparty -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    PREFIX ${THIRD_PARTY_DIR}/lion
    SOURCE_DIR "${THIRD_PARTY_DIR}/lion/source"
    BINARY_DIR "${THIRD_PARTY_DIR}/lion/build"
    INSTALL_DIR ${THIRD_PARTY_DIR}
)
endif()
