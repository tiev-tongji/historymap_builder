#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cartographer" for configuration "Release"
set_property(TARGET cartographer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cartographer PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcartographer.so"
  IMPORTED_SONAME_RELEASE "libcartographer.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cartographer )
list(APPEND _IMPORT_CHECK_FILES_FOR_cartographer "${_IMPORT_PREFIX}/lib/libcartographer.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
