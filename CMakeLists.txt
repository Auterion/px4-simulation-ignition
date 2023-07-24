cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_gazebosim)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

set(GAZEBO_VERSION 7)
find_package(gz-sim7 REQUIRED)
find_package(gz-msgs9)
find_package(gz-sensors7 REQUIRED)
if(gz-msgs9_FOUND)
  set(GZ_MSG_VER 9)
  set(Protobuf_IMPORT_DIRS ${gz-msgs9_INCLUDE_DIRS})
else()
  set(GZ_MSG_VER 8)
  find_package(gz-msgs8 REQUIRED)
  set(Protobuf_IMPORT_DIRS ${gz-msgs8_INCLUDE_DIRS})
endif()

find_package(MAVLink)

find_package(Protobuf REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${MAVLINK_INCLUDE_DIRS}
)

add_library(mavlink_sitl_gazebosim SHARED src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp)
set_property(TARGET mavlink_sitl_gazebosim PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_gazebosim
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
  PRIVATE gz-sensors7::gz-sensors7
)

include(GNUInstallDirs)
install(TARGETS
  mavlink_sitl_gazebosim
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)
