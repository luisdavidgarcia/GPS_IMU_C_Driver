add_library(gps gps.cpp)

target_include_directories(gps PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(gps PRIVATE ubx)