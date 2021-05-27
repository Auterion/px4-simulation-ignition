cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_ign_gazebo)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo5 REQUIRED)

find_package(MAVLink)


set(Protobuf_IMPORT_DIRS ${ignition-msgs7_INCLUDE_DIRS})
find_package(Protobuf REQUIRED)
add_subdirectory(msgs)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
                           PROPERTIES GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${MAVLINK_INCLUDE_DIRS}
)

add_library(mavlink_sitl_ign_gazebo SHARED src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp)
set_property(TARGET mavlink_sitl_ign_gazebo PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_ign_gazebo
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5
  PRIVATE sensor_msgs
)

add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
set_property(TARGET gazebo_barometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_barometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5
  PRIVATE sensor_msgs
)

add_library(gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp src/geo_mag_declination.cpp)
set_property(TARGET gazebo_magnetometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_magnetometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5
  PRIVATE sensor_msgs
)

add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
set_property(TARGET gazebo_gps_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_gps_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo5::ignition-gazebo5
  PRIVATE sensor_msgs
)
