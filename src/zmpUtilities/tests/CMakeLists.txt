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
  set(CMAKE_INSTALL_PREFIX /usr/local )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
endif()

set(CMAKE_DEBUG_POSTFIX "d")


###############
# Link with DART and GRIP
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

######################
# Set vrc-golem path
set( VRC_GOLEM_PATH /home/ana/Software/golems/vrc-golem )

include_directories( ${DARTExt_INCLUDEDIR} )
include_directories( ${DART_INCLUDEDIR} )
include_directories( ${VRC_GOLEM_PATH}/dart-atlas/src  .  .. )

link_directories(${DART_LIBRARY_DIRS})
link_directories( ${VRC_GOLEM_PATH}/dart-atlas/lib )

# Search all header and source files

add_executable( generateTraj generateTraj.cpp )
target_link_libraries( generateTraj  zmpUtilities dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
add_executable( testFootprints testFootprints.cpp ${zmp_src} ${zmp_hdr} )
target_link_libraries( testFootprints  zmpUtilities tinyWalker  dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
