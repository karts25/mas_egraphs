cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(mas_egraphs src/egraph_xy.cpp
                                 src/mas_egraph.cpp
				 src/environment_xy.cpp
				 src/egraph_planner.hpp
				 src/egraphManager.hpp					
				 src/egraph_mas_2d_grid_heuristic.cpp
				 )
target_link_libraries(mas_egraphs ${SBPL_LIBRARIES})
rosbuild_add_executable(sensorNode src/sensor_node.cpp)
rosbuild_add_executable(egraphXYNode src/egraph_xy_node.cpp)
#rosbuild_add_executable(interactXYTheta src/interactXYTheta.cpp)
target_link_libraries(egraphXYNode mas_egraphs)
#rosbuild_add_executable(interactXYTheta src/interactXYTheta.cpp)
rosbuild_add_executable(runTests src/run_tests.cpp)
