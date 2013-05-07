cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
    src/BSpline.cpp
    src/Function.cpp
    src/SplinePath.cpp
    src/SplineRegister.cpp
)
target_link_libraries(${PROJECT_NAME} gsl gslcblas)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(test1 test/test.cpp)
target_link_libraries(test1 ${PROJECT_NAME})

rosbuild_add_executable(testsp test/testsp.cpp)
target_link_libraries(testsp ${PROJECT_NAME})

rosbuild_add_executable(optimsp test/optimsp.cpp)
target_link_libraries(optimsp ${PROJECT_NAME})

rosbuild_add_executable(function test/function.cpp)
target_link_libraries(function ${PROJECT_NAME})

rosbuild_add_executable(travmaps test/travmaps.cpp)
target_link_libraries(travmaps ${PROJECT_NAME})
