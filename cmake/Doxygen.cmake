# Doxygen support
# adapted from http://www.bluequartz.net/projects/EIM_Segmentation/
#    SoftwareDocumentation/html/usewithcmakeproject.html
include(CMakeDependentOption)
OPTION(BUILD_DOC "Build source code documentation using doxygen" OFF)

CMAKE_DEPENDENT_OPTION(BUILD_DOC_QHP 
    "Use Doxygen to create documentation for Qt Creator" OFF
    "BUILD_DOC" OFF)
CMAKE_DEPENDENT_OPTION(BUILD_DOC_XCODE 
    "Use Doxygen to create documentation for Apple Xcode" OFF
    "BUILD_DOC" OFF)
CMAKE_DEPENDENT_OPTION(BUILD_DOC_ECLIPSE
    "Use Doxygen to create documentation for Eclipse" OFF
    "BUILD_DOC" OFF)
CMAKE_DEPENDENT_OPTION(BUILD_DOC_FIXEDWIDTH
    "Use a fixed-width style sheet for doxygen output" OFF
    "BUILD_DOC" OFF)

IF (BUILD_DOC)
    FIND_PACKAGE(Doxygen)
    IF(NOT DOXYGEN_FOUND)
        MESSAGE(WARNING 
                "Doxygen not found. Building the documentation will fail.")
    ENDIF()
    IF(BUILD_DOC_QHP)
        SET(DOXYGEN_GENERATE_QHP "YES")
    ENDIF()
    IF(BUILD_DOC_XCODE)
        SET(DOXYGEN_GENERATE_DOCSET "YES")
    ENDIF()
    IF(BUILD_DOC_ECLIPSE)
        SET(DOXYGEN_GENERATE_ECLIPSEHELP "YES")
    ENDIF()

    SET(DOXYGEN_EXTRA_CSS "")
#   IF(BUILD_DOC_FIXEDWIDTH)
#       SET(DOXYGEN_EXTRA_CSS "docs/doxygen/custom.css docs/doxygen/custom_dark_theme.css")
#   ENDIF()

    INSTALL(DIRECTORY ${PROJECT_BINARY_DIR}/docs/doxygen/html/
        DESTINATION ${DOC_DIR}/doxygen)

    IF(DOXYGEN_DOT_EXECUTABLE)
        MESSAGE(STATUS "Doxygen dot executable found. Enabling graph generation in docs...")
        SET(DOXYGEN_DOT_AVAILABLE "YES")
    ELSE()
        MESSAGE(STATUS "Doxygen dot executable NOT found. Disabling graph generation in docs...")
        SET(DOXYGEN_DOT_AVAILABLE "NO")
    ENDIF()
    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile.in
                   ${PROJECT_BINARY_DIR}/docs/doxygen/Doxyfile @ONLY)

    ADD_CUSTOM_TARGET(doc
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/docs/doxygen/custom.css
            ${PROJECT_BINARY_DIR}/docs/doxygen/custom.css
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/docs/doxygen/custom_dark_theme.css
            ${PROJECT_BINARY_DIR}/docs/doxygen/custom_dark_theme.css
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/docs/doxygen/html_header.html
            ${PROJECT_BINARY_DIR}/docs/doxygen/html_header.html
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/docs/doxygen/html_footer.html
            ${PROJECT_BINARY_DIR}/docs/doxygen/html_footer.html
        COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/docs/doxygen/Doxyfile
        SOURCES ${PROJECT_BINARY_DIR}/docs/doxygen/Doxyfile)

ENDIF (BUILD_DOC)

