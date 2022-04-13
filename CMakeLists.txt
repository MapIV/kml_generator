cmake_minimum_required(VERSION 2.8.3)
project(kml_generator)

# Enable support for C++11
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  set(CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  LIBRARIES kml_generator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kml_generator src/kml_generator/kml_generator.cpp)
target_link_libraries(kml_generator ${catkin_LIBRARIES})
add_dependencies(kml_generator ${catkin_EXPORTED_TARGETS})
