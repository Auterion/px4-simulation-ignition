cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_ign_gazebo)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo3 REQUIRED)

set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

include_directories(include)
add_library(mavlink_sitl_ign_gazebo SHARED src/gazebo_mavlink_interface.cpp)
set_property(TARGET mavlink_sitl_ign_gazebo PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_ign_gazebo
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo3::ignition-gazebo3)
