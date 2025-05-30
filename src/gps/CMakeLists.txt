add_library(GPS GPS.cpp)

target_include_directories(GPS PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(GPS PRIVATE UBX)