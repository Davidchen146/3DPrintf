#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lexers" for configuration "Release"
set_property(TARGET lexers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lexers PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblexers.a"
  )

list(APPEND _cmake_import_check_targets lexers )
list(APPEND _cmake_import_check_files_for_lexers "${_IMPORT_PREFIX}/lib/liblexers.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
