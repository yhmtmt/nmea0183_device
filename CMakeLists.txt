protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_SOURCE_DIR} decoder_config.proto)
add_library(nmea0183_device SHARED f_nmea0183_device.cpp ${CHANS} ${PROTO_SRCS})

target_include_directories(nmea0183_device PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(nmea0183_device PUBLIC ${PROJECT_SOURCE_DIR}/channels)

install(TARGETS nmea0183_device DESTINATION lib)
