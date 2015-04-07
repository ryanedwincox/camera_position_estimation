cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/build)
set(POD_NAME apriltags)
include(build/pods.cmake)

file(GLOB SOURCE_FILES "/usr/include/apriltags/src/*.cc")
#set(GLOB SOURCE_FILES "src/*.cc")
include_directories("/usr/include/apriltags/AprilTags")
add_library(apriltags ${SOURCE_FILES})

pods_install_libraries(apriltags)

file(GLOB header_files "/usr/include/apriltags/AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltags ${OpenCV_LIBS})
pods_use_pkg_config_packages(apriltags eigen3)

include_directories("/usr/local/cuda-6.5/include")
include_directories("/opt/AMDAPP/include")
include_directories("/usr/include/")

set(SOURCE_FILES
    src/positionEstimation.cpp
    src/april.h
    src/april.cpp
    manifest.xml
)

rosbuild_add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} OpenCL)

pods_install_pkg_config_file(apriltags
    LIBS -lapriltags
    REQUIRES eigen3 opencv2
    VERSION 1.0.0)
