add_library(nmea0183_device SHARED f_nmea0183_device.cpp ${CHANS})

target_include_directories(nmea0183_device PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(nmea0183_device PUBLIC ${PROJECT_SOURCE_DIR}/channels)

install(TARGETS nmea0183_device DESTINATION lib)
