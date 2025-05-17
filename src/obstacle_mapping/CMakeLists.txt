cmake_minimum_required(VERSION 3.0.2)
project(obstacle_mapping)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  grid_map_core
  grid_map_ros
  grid_map_msgs
  geometry_msgs
  visualization_msgs
)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

pkg_check_modules(TBB "tbb")
if (TBB_FOUND)
  add_definitions(
    ${TBB_CFLAGS}
    ${TBB_CFLAGS_OTHER}
  )
else()
  message([FATAL_ERROR] "tbb module not found")
endif ()

catkin_package(
  INCLUDE_DIRS 
    include
    ${TBB_INCLUDE_DIRS}
  LIBRARIES obstacle_mapping_lib
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    std_msgs
  DEPENDS 
    system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(obstacle_mapping_lib
  src/obstacle_mapping.cpp
)
target_link_libraries(obstacle_mapping_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${TBB_LIBRARIES})

add_executable(obstacle_mapping_node src/obstacle_mapping_node.cpp)
add_dependencies(obstacle_mapping_node obstacle_mapping_lib)
target_link_libraries(obstacle_mapping_node ${catkin_LIBRARIES} obstacle_mapping_lib)

add_executable(pose_prediction_node src/pose_prediction_node.cpp)
target_link_libraries(pose_prediction_node ${catkin_LIBRARIES})

add_executable(pose_prediction src/pose_prediction.cpp)
target_link_libraries(pose_prediction ${catkin_LIBRARIES})

add_executable(key_pub_node src/key_pub_node.cpp)
target_link_libraries(key_pub_node ${catkin_LIBRARIES})

add_executable(gridmap_saver_node src/gridmap_saver.cpp)
target_link_libraries(gridmap_saver_node ${catkin_LIBRARIES})

add_executable(show_vehicle_node src/show_vehicle_node.cpp)
target_link_libraries(show_vehicle_node ${catkin_LIBRARIES})

add_executable(recieve_points src/recieve_points.cpp)
target_link_libraries(recieve_points ${catkin_LIBRARIES})

add_executable(publish_points src/publish_points.cpp)
target_link_libraries(publish_points ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS obstacle_mapping_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS obstacle_mapping
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/obstacle_mapping/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(obstacle_mapping-test test/test_obstacle_mapping.cpp)
# if(TARGET obstacle_mapping-test)
#   target_link_libraries(obstacle_mapping-test obstacle_mapping)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
