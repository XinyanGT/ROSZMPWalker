# tinyWalker Library
include_directories (.. )
add_library( tinyWalker zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp ) 
set_target_properties( tinyWalker PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )



