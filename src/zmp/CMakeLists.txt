#cmake_minimum_required (VERSION 2.8)
#project (walkPlugin)

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


###############
# Link with DART
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
#include_directories( /home/yan/Course/Robotics/vrc-golem/dart-atlas/src . )
#include_directories( /home/ana/Research/HUBO/vrc-golem/dart-atlas/src . )
include_directories(/home/eric/ros/ROSZMPWalker/src)

link_directories(${DART_LIBRARY_DIRS})
#link_directories( /home/yan/Course/Robotics/vrc-golem/dart-atlas/lib )
#link_directories(/home/ana/Research/HUBO/vrc-golem/dart-atlas/lib )

# Add plugin
#rosbuild_add_executable( walk  walk.cpp zmp/zmpUtilities.cpp )
rosbuild_add_executable( zmp_walk zmp_walk.cpp ../walk/zmp/zmpUtilities.cpp main.cpp)

#target_link_libraries( walk ${DART_LIBRARIES} ${DARTExt_LIBRARIES} dart_atlas )
target_link_libraries( zmp_walk ${DART_LIBRARIES} ${DARTExt_LIBRARIES} dart_atlas )
