add_library(nmea0183_device SHARED f_nmea0183_device.cpp)

target_include_directories(nmea0183_device PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS nmea0183_device DESTINATION lib)
