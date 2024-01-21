# FLAGS=-li2c -ggdb

# all: imu_test gps_test kalman_test

# imu_test:
# 	g++ imu_module/imu.cpp tests/imu_tests/test_imu.cpp -o test_imu $(FLAGS)

# gps_test:
# 	g++ gps_module/gps.cpp ubx_lib/ubx_msg.cpp tests/gps_tests/test_gps.cpp -o test_gps $(FLAGS)

# kalman_test:
# 	g++ -std=c++14 imu_module/imu.cpp gps_module/gps.cpp ubx_lib/ubx_msg.cpp tests/kalman_tests/test_kalman.cpp -o test_ekf ekfNavINS.o $(FLAGS) -I /usr/include/eigen3

# graphing:
# 	g++ -std=c++17 basic.cpp -o basic -lmatplot -I /usr/local/include/matplot

# gps_map_test:
# 	g++ gps_module/gps.cpp ubx_lib/ubx_msg.cpp tests/gps_tests/gps_map.cpp -o gps_map_test $(FLAGS) -lcurl

# clean:
# 	rm -rf test_imu test_gps

CXX=g++
CXXFLAGS=-ggdb -I /usr/include/eigen3
LDFLAGS=-li2c
LIBS=-lmatplot -lcurl
GPS_DEPS=ubx_lib/ubx_msg.cpp
OBJ_DIR=obj
CROSS_COMPILE_CXX=aarch64-linux-gnu-g++
CROSS_COMPILE_FLAGS=-std=c++14 -I /usr/include/eigen3

all: imu_test gps_test kalman_test graphing gps_map_test

$(OBJ_DIR)/%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(OBJ_DIR)/imu.o: imu_module/imu.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(OBJ_DIR)/gps.o: gps_module/gps.cpp $(GPS_DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

ekfNavINS.o: extended_kalman_filter/ekfNavINS.cpp
	$(CROSS_COMPILE_CXX) $(CROSS_COMPILE_FLAGS) -c extended_kalman_filter/ekfNavINS.cpp -o ekfNavINS.o

imu_test: $(OBJ_DIR)/imu.o
	$(CXX) $(OBJ_DIR)/imu.o tests/imu_tests/test_imu.cpp -o test_imu $(CXXFLAGS) $(LDFLAGS)

gps_test: $(OBJ_DIR)/gps.o
	$(CXX) $(OBJ_DIR)/gps.o $(GPS_DEPS) tests/gps_tests/test_gps.cpp -o test_gps $(CXXFLAGS) $(LDFLAGS)

kalman_test: $(OBJ_DIR)/imu.o $(OBJ_DIR)/gps.o ekfNavINS.o
	$(CXX) $(OBJ_DIR)/imu.o $(OBJ_DIR)/gps.o ekfNavINS.o tests/kalman_tests/test_kalman.cpp -o test_ekf $(CXXFLAGS) $(LDFLAGS)

graphing:
	$(CXX) -std=c++17 basic.cpp -o basic $(CXXFLAGS) $(LIBS)

gps_map_test: $(OBJ_DIR)/gps.o
	$(CXX) $(OBJ_DIR)/gps.o $(GPS_DEPS) tests/gps_tests/gps_map.cpp -o gps_map_test $(CXXFLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(OBJ_DIR)/*.o test_imu test_gps test_ekf basic gps_map_test ekfNavINS.o
