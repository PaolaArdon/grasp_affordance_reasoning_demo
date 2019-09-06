cmake_minimum_required(VERSION 2.8.3)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./include/)

add_library(utils src/sq_utils.cpp)
add_library(fitting src/sq_fitting.cpp)
add_library(sampling src/sq_sampling.cpp)
add_library(segmentation src/scene_segmentation.cpp)

target_link_libraries(utils ${PCL_LIBRARY_DIRS})
target_link_libraries(fitting utils ${PCL_LIBRARY_DIRS})
target_link_libraries(sampling utils ${PCL_LIBRARY_DIRS})
target_link_libraries(segmentation ${PCL_LIBRARY_DIRS})

add_executable(viewer src/cloud_viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})

add_executable(get_sq src/scene_as_sqs.cpp)
target_link_libraries(get_sq fitting sampling segmentation ${PCL_LIBRARIES})
