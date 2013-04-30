# CMake for atlaszmp library

# Include PkgConfig
include(FindPkgConfig)

#Find Eigen
pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Include directories
include_directories( src . )

add_subdirectory(zmp)


