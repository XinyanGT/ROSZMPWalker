# Create zmpUtilities library, depending on tinyWalker

#Generate tinyWalker
add_subdirectory( tinyWalker )
include_directories( . tinyWalker /home/ana/Research/ROBOT/ROSZMPWalker/src/zmpUtilities/tinyWalker )

add_library( zmpUtilities zmpUtilities.cpp zmpUtilities.h )
target_link_libraries( zmpUtilities tinyWalker )
set_target_properties( zmpUtilities PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
