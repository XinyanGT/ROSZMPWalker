cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



###############
# Compile flags
if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /openmp")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set(CMAKE_INSTALL_PREFIX /usr )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
endif()

set(CMAKE_DEBUG_POSTFIX "d")




# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# find gazebo 
include(FindPkgConfig)
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
else()
  message( FATAL_ERROR "pkg-config is required; please install it" )
endif()

# depends on DRCVehiclePlugin
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} ) 




###############
# Link with DART
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories( /home/eric/ros/vrc-golem/dart-atlas/src . )
include_directories(/home/eric/ros/ROSZMPWalker/src)

link_directories(${DART_LIBRARY_DIRS})
link_directories( /home/eric/ros/vrc-golem/dart-atlas/lib )
link_directories( /home/eric/golems/dart/lib)


# Eigen
include_directories( /usr/local/include/eigen3/ )

rosbuild_add_executable( publish_joint_commands src/publish_joint_commands.cpp )
rosbuild_add_executable( publish_joint_command_fromTxt src/publish_joint_commands_fromTxt.cpp )
#rosbuild_add_executable( publish_and_readonce src/publish_and_readonce.cpp )
rosbuild_add_executable( subscribe_jointStates_test src/subscribe_jointStates_test.cpp )
rosbuild_add_executable( test_keyframes src/test_keyframes.cpp )

# Eric
rosbuild_add_executable( zmp_walk src/zmp/zmp_walk.cpp src/walk/zmp/zmpUtilities.cpp src/zmp/main.cpp)
target_link_libraries( zmp_walk dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

# Walk
add_subdirectory( src/walk )

