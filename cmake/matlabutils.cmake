#[=======================================================================[.rst:
.. command:: matlab_add_mex

  Adds a Matlab MEX target.
  This commands compiles the given sources with the current tool-chain in
  order to produce a MEX file. The final name of the produced output may be
  specified, as well as additional link libraries, and a documentation entry
  for the MEX file. Remaining arguments of the call are passed to the
  :command:`add_library` or :command:`add_executable` command.

  ::

     matlab_add_mex(
         NAME <name>
         [EXECUTABLE | MODULE | SHARED]
         SRC src1 [src2 ...]
         [OUTPUT_NAME output_name]
         [DOCUMENTATION file.txt]
         [LINK_TO target1 target2 ...]
         [R2017b | R2018a]
         [EXCLUDE_FROM_ALL]
         [...]
     )

  ``NAME``
    name of the target.
  ``SRC``
    list of source files.
  ``LINK_TO``
    a list of additional link dependencies.  The target links to ``libmex``
    and ``libmx`` by default.
  ``OUTPUT_NAME``
    if given, overrides the default name. The default name is
    the name of the target without any prefix and
    with ``Matlab_MEX_EXTENSION`` suffix.
  ``DOCUMENTATION``
    if given, the file ``file.txt`` will be considered as
    being the documentation file for the MEX file. This file is copied into
    the same folder without any processing, with the same name as the final
    mex file, and with extension `.m`. In that case, typing ``help <name>``
    in Matlab prints the documentation contained in this file.
  ``R2017b`` or ``R2018a``
    .. versionadded:: 3.14

    May be given to specify the version of the C API
    to use: ``R2017b`` specifies the traditional (separate complex) C API,
    and corresponds to the ``-R2017b`` flag for the `mex` command. ``R2018a``
    specifies the new interleaved complex C API, and corresponds to the
    ``-R2018a`` flag for the `mex` command. Ignored if MATLAB version prior
    to R2018a. Defaults to ``R2017b``.

  ``MODULE`` or ``SHARED``
    .. versionadded:: 3.7

    May be given to specify the type of library to be
    created.

  ``EXECUTABLE``
    .. versionadded:: 3.7

    May be given to create an executable instead of
    a library. If no type is given explicitly, the type is ``SHARED``.
  ``EXCLUDE_FROM_ALL``
    This option has the same meaning as for :prop_tgt:`EXCLUDE_FROM_ALL` and
    is forwarded to :command:`add_library` or :command:`add_executable`
    commands.

  The documentation file is not processed and should be in the following
  format:

  ::

    % This is the documentation
    function ret = mex_target_output_name(input1)

#]=======================================================================]
function(my_matlab_add_mex)

  if(NOT WIN32)
    # we do not need all this on Windows
    # pthread options
    if(CMAKE_CXX_COMPILER_LOADED)
      check_cxx_compiler_flag(-pthread HAS_MINUS_PTHREAD)
    elseif(CMAKE_C_COMPILER_LOADED)
      check_c_compiler_flag(-pthread HAS_MINUS_PTHREAD)
    endif()
    # we should use try_compile instead, the link flags are discarded from
    # this compiler_flag function.
    #check_cxx_compiler_flag(-Wl,--exclude-libs,ALL HAS_SYMBOL_HIDING_CAPABILITY)

  endif()

  set(options EXECUTABLE MODULE SHARED R2017b R2018a EXCLUDE_FROM_ALL)
  set(oneValueArgs NAME DOCUMENTATION OUTPUT_NAME)
  set(multiValueArgs LINK_TO SRC)

  set(prefix _matlab_addmex_prefix)
  cmake_parse_arguments(${prefix} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  if(NOT ${prefix}_NAME)
    message(FATAL_ERROR "[MATLAB] The MEX target name cannot be empty")
  endif()

  if(NOT ${prefix}_OUTPUT_NAME)
    set(${prefix}_OUTPUT_NAME ${${prefix}_NAME})
  endif()

  if(NOT Matlab_VERSION_STRING VERSION_LESS "9.1") # For 9.1 (R2016b) and newer, add version source file
    # Add the correct version file depending on which languages are enabled in the project
    if(CMAKE_C_COMPILER_LOADED)
      # If C is enabled, use the .c file as it will work fine also with C++
      set(MEX_VERSION_FILE "${Matlab_ROOT_DIR}/extern/version/c_mexapi_version.c")
    elseif(CMAKE_CXX_COMPILER_LOADED)
      # If C is not enabled, check if CXX is enabled and use the .cpp file
      # to avoid that the .c file is silently ignored
      set(MEX_VERSION_FILE "${Matlab_ROOT_DIR}/extern/version/cpp_mexapi_version.cpp")
    else()
      # If neither C or CXX is enabled, warn because we cannot add the source.
      # TODO: add support for fortran mex files
      message(WARNING "[MATLAB] matlab_add_mex requires that at least C or CXX are enabled languages")
    endif()
  endif()

  # For 9.4 (R2018a) and newer, add API macro.
  # Add it for unknown versions too, just in case.
  if(NOT Matlab_VERSION_STRING VERSION_LESS "9.4"
      OR Matlab_VERSION_STRING STREQUAL "unknown")
    if(${${prefix}_R2018a})
      set(MEX_API_MACRO "MATLAB_DEFAULT_RELEASE=R2018a")
    else()
      set(MEX_API_MACRO "MATLAB_DEFAULT_RELEASE=R2017b")
    endif()
  endif()

  set(_option_EXCLUDE_FROM_ALL)
  if(${prefix}_EXCLUDE_FROM_ALL)
    set(_option_EXCLUDE_FROM_ALL "EXCLUDE_FROM_ALL")
  endif()

  if(${prefix}_EXECUTABLE)
    add_executable(${${prefix}_NAME}
      ${_option_EXCLUDE_FROM_ALL}
      ${${prefix}_SRC}
      ${MEX_VERSION_FILE}
      ${${prefix}_DOCUMENTATION}
      ${${prefix}_UNPARSED_ARGUMENTS})
  else()
    if(${prefix}_MODULE)
      set(type MODULE)
    else()
      set(type SHARED)
    endif()

    add_library(${${prefix}_NAME}
      ${type}
      ${_option_EXCLUDE_FROM_ALL}
      ${${prefix}_SRC}
      ${MEX_VERSION_FILE}
      ${${prefix}_DOCUMENTATION}
      ${${prefix}_UNPARSED_ARGUMENTS})
  endif()

  target_include_directories(${${prefix}_NAME} PRIVATE ${Matlab_INCLUDE_DIRS})

  if(Matlab_HAS_CPP_API)
    if(Matlab_ENGINE_LIBRARY)
      target_link_libraries(${${prefix}_NAME} ${Matlab_ENGINE_LIBRARY})
    endif()
    if(Matlab_DATAARRAY_LIBRARY)
      target_link_libraries(${${prefix}_NAME} ${Matlab_DATAARRAY_LIBRARY})
    endif()
  endif()

  target_link_libraries(${${prefix}_NAME} ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} ${${prefix}_LINK_TO})
  set_target_properties(${${prefix}_NAME}
      PROPERTIES
        PREFIX ""
        OUTPUT_NAME ${${prefix}_OUTPUT_NAME}
        SUFFIX ".${Matlab_MEX_EXTENSION}")

  target_compile_definitions(${${prefix}_NAME} PRIVATE ${MEX_API_MACRO} MATLAB_MEX_FILE)

  # documentation
  if(NOT ${${prefix}_DOCUMENTATION} STREQUAL "")
    get_target_property(output_name ${${prefix}_NAME} OUTPUT_NAME)
    add_custom_command(
      TARGET ${${prefix}_NAME}
      PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${${prefix}_DOCUMENTATION} $<TARGET_FILE_DIR:${${prefix}_NAME}>/${output_name}.m
      COMMENT "[MATLAB] Copy ${${prefix}_NAME} documentation file into the output folder"
    )
  endif() # documentation

  # entry point in the mex file + taking care of visibility and symbol clashes.
  if(WIN32)

    if (MSVC)

      set(_link_flags "${_link_flags} /EXPORT:mexFunction")
      if(NOT Matlab_VERSION_STRING VERSION_LESS "9.1") # For 9.1 (R2016b) and newer, export version
        set(_link_flags "${_link_flags} /EXPORT:mexfilerequiredapiversion")
      endif()

      set_property(TARGET ${${prefix}_NAME} APPEND PROPERTY LINK_FLAGS ${_link_flags})

    endif() # No other compiler currently supported on Windows.

    set_target_properties(${${prefix}_NAME}
      PROPERTIES
        DEFINE_SYMBOL "DLL_EXPORT_SYM=__declspec(dllexport)")

  else()

    if(Matlab_VERSION_STRING VERSION_LESS "9.1") # For versions prior to 9.1 (R2016b)
      set(_ver_map_files ${Matlab_EXTERN_LIBRARY_DIR}/mexFunction.map)
    else()                                          # For 9.1 (R2016b) and newer
      set(_ver_map_files ${Matlab_EXTERN_LIBRARY_DIR}/c_exportsmexfileversion.map)
    endif()

    if(NOT Matlab_VERSION_STRING VERSION_LESS "9.5") # For 9.5 (R2018b) (and newer?)
      target_compile_options(${${prefix}_NAME} PRIVATE "-fvisibility=default")
      # This one is weird, it might be a bug in <mex.h> for R2018b. When compiling with
      # -fvisibility=hidden, the symbol `mexFunction` cannot be exported. Reading the
      # source code for <mex.h>, it seems that the preprocessor macro `MW_NEEDS_VERSION_H`
      # needs to be defined for `__attribute__((visibility("default")))` to be added
      # in front of the declaration of `mexFunction`. In previous versions of MATLAB this
      # was not the case, there `DLL_EXPORT_SYM` needed to be defined.
      # Adding `-fvisibility=hidden` to the `mex` command causes the build to fail.
      # TODO: Check that this is still necessary in R2019a when it comes out.
    endif()

    if(APPLE)

      if(Matlab_HAS_CPP_API)
        list(APPEND _ver_map_files ${Matlab_EXTERN_LIBRARY_DIR}/cppMexFunction.map) # This one doesn't exist on Linux
        set(_link_flags "${_link_flags} -Wl,-U,_mexCreateMexFunction -Wl,-U,_mexDestroyMexFunction -Wl,-U,_mexFunctionAdapter")
        # On MacOS, the MEX command adds the above, without it the link breaks
        # because we indiscriminately use "cppMexFunction.map" even for C API MEX-files.
      endif()

      set(_export_flag_name -exported_symbols_list)

    else() # Linux

      if(HAS_MINUS_PTHREAD)
        # Apparently, compiling with -pthread generated the proper link flags
        # and some defines at compilation
        target_compile_options(${${prefix}_NAME} PRIVATE "-pthread")
      endif()

      set(_link_flags "${_link_flags} -Wl,--as-needed")

      set(_export_flag_name --version-script)

    endif()

    foreach(_file ${_ver_map_files})
      set(_link_flags "${_link_flags} -Wl,${_export_flag_name},${_file}")
    endforeach()

    # The `mex` command doesn't add this define. It is specified here in order
    # to export the symbol in case the client code decides to hide its symbols
    set_target_properties(${${prefix}_NAME}
      PROPERTIES
#       DEFINE_SYMBOL "DLL_EXPORT_SYM=__attribute__((visibility(\"default\")))"
        LINK_FLAGS "${_link_flags}"
    )

  endif()

endfunction()
