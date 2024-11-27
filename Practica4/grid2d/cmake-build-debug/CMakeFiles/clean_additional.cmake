# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "src/CMakeFiles/grid2d_autogen.dir/AutogenUsed.txt"
  "src/CMakeFiles/grid2d_autogen.dir/ParseCache.txt"
  "src/grid2d_autogen"
  )
endif()
