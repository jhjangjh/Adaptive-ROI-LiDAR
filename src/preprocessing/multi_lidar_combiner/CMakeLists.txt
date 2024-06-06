cmake_minimum_required(VERSION 3.0.2)
project(multi_lidar_combiner_pkg)
set(CMAKE_BUILD_TYPE Release)

set ( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES opencv
 CATKIN_DEPENDS roscpp pcl_ros pcl_conversions std_msgs sensor_msgs
 DEPENDS system_lib OpenMP
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})


include_directories(
	include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


catkin_package(CATKIN_DEPENDS)
set(BUILD_surface_on_nurbs)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(multi_lidar_combiner_node node/multi_lidar_combiner_node.cpp
               src/multi_lidar_combiner.cpp
)

add_dependencies(multi_lidar_combiner_node ${catkin_EXPORTED_TARGETS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(multi_lidar_combiner_node 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES}
                      ${OpenMP_LIBS}
                      ${YAML_CPP_LIBRARIES}
)

target_compile_options(multi_lidar_combiner_node PRIVATE ${OpenMP_FLAGS})
