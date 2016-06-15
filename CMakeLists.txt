cmake_minimum_required(VERSION 3.0)

project(voxelserver)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/vendor/libamf/CMakeModules/")

add_definitions(
  -DUNORDERED
)

set(includes
	include/
	#build/
)

file(GLOB sources
	src/*.cpp
)

add_subdirectory(src/vendor)

message("----------------------------------------------------------------------")

set(names "Includes:\n  ")
foreach(include ${includes})
  #get_filename_component(filename "${include}" NAME)
  set(names "${names} ${include}\n  ")
endforeach()
message("${names}")

set(names "Sources:\n  ")
foreach(source ${sources})
  get_filename_component(filename "${source}" NAME)
  set(names "${names} ${filename} |")
endforeach()
message("${names}")

message("----------------------------------------------------------------------")

add_executable(voxelserver ${sources})

target_include_directories(voxelserver PRIVATE ${includes})
