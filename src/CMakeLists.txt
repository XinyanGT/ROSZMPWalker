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

#################################
# Compile zmpUtilities library
add_subdirectory( zmpUtilities )

#################
# Plugins
include_directories( . zmpUtilities/tinyWalker )

# Eric code
#rosbuild_add_executable( zmp_walk zmp/zmp_walk.cpp zmp/main.cpp)
#target_link_libraries( zmp_walk dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

# Ana and Xinyan's code
# Add plugin that works with DRCSim 2.2
#rosbuild_add_executable( publish_and_readonce publish_and_readonce.cpp )
#target_link_libraries( publish_and_readonce ${DART_LIBRARIES} ${DARTExt_LIBRARIES} dart_atlas zmpUtilities )

# Plugin that works in DRCSim 2.5 and is cleaner
rosbuild_add_executable( johnnieWalker johnnieWalker.cpp )
target_link_libraries( johnnieWalker ${DART_LIBRARIES} ${DARTExt_LIBRARIES} dart_atlas zmpUtilities )
