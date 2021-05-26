set(Protobuf_IMPORT_DIRS ${ignition-msgs7_INCLUDE_DIRS})
find_package(Protobuf REQUIRED)

set(sensor_msgs
  Pressure.proto
  MagneticField.proto
  Groundtruth.proto
  SITLGps.proto
  )

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

# Variables needed to propagate through modules
# If more than one layer of cmake use CACHE INTERNAL ...
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)

add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})
link_libraries(sensor_msgs)
target_link_libraries(sensor_msgs ignition-msgs7)
