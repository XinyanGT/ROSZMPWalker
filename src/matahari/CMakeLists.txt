##########################
# Matahari

# Build library adaptedZMP
add_subdirectory( adaptedZMP )
include_directories( . )
rosbuild_add_executable( matahari matahari.cpp )
target_link_libraries( matahari ${DART_LIBRARIES} ${DARTExt_LIBRARIES} dart_atlas adaptedZMP )