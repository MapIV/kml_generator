cmake_minimum_required(VERSION 3.14)
project(kml_generator)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

ament_auto_add_library(kml_generator STATIC
  src/kml_generator/kml_generator_core.cpp
  src/kml_generator/coordinate_transformation.cpp
)
ament_export_include_directories("include/kml_generator")

ament_auto_package()
