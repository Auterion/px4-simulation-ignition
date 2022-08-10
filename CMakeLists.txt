cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_ign_gazebo)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo5)
if(ignition-gazebo5_FOUND)
  set(GAZEBO_VERSION 5)
  set(IGN_MSG_VER 7)
  find_package(ignition-msgs7 REQUIRED)
  set(Protobuf_IMPORT_DIRS ${ignition-msgs7_INCLUDE_DIRS})
else()
  set(GAZEBO_VERSION 6)
  set(IGN_MSG_VER 8)
  find_package(ignition-msgs8 REQUIRED)
  find_package(ignition-gazebo6 REQUIRED)
  set(Protobuf_IMPORT_DIRS ${ignition-msgs8_INCLUDE_DIRS})
endif()

find_package(MAVLink)

find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS
  msgs/Pressure.proto
  msgs/MagneticField.proto
  msgs/Groundtruth.proto
  msgs/SITLGps.proto
  )
set_source_files_properties(${SEN_PROTO_SRC} ${SEN_PROTO_HDRS}
                           PROPERTIES GENERATED TRUE)
add_library(gazebo_mavlink_sensor_msgs SHARED ${SEN_PROTO_SRCS})
target_link_libraries(gazebo_mavlink_sensor_msgs ignition-msgs${IGN_MSG_VER})

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
  PRIVATE ignition-gazebo${GAZEBO_VERSION}::ignition-gazebo${GAZEBO_VERSION}
  PRIVATE gazebo_mavlink_sensor_msgs
)

add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
set_property(TARGET gazebo_barometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_barometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${GAZEBO_VERSION}::ignition-gazebo${GAZEBO_VERSION}
  PRIVATE gazebo_mavlink_sensor_msgs
)

add_library(gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp src/geo_mag_declination.cpp)
set_property(TARGET gazebo_magnetometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_magnetometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${GAZEBO_VERSION}::ignition-gazebo${GAZEBO_VERSION}
  PRIVATE gazebo_mavlink_sensor_msgs
)

add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
set_property(TARGET gazebo_gps_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_gps_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${GAZEBO_VERSION}::ignition-gazebo${GAZEBO_VERSION}
  PRIVATE gazebo_mavlink_sensor_msgs
)

include(GNUInstallDirs)
install(TARGETS
  mavlink_sitl_ign_gazebo
  gazebo_barometer_plugin
  gazebo_magnetometer_plugin
  gazebo_gps_plugin
  gazebo_mavlink_sensor_msgs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)
