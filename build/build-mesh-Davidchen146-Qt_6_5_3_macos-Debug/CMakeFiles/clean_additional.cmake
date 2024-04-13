# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/mesh_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/mesh_autogen.dir/ParseCache.txt"
  "mesh_autogen"
  )
endif()
