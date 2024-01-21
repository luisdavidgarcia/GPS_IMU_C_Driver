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
CXXFLAGS=-ggdb -I /usr/include/eigen3 -I include/
LDFLAGS=-li2c
LIBS=-lmatplot -lcurl
OBJ_DIR=obj
CROSS_COMPILE_CXX=aarch64-linux-gnu-g++
CROSS_COMPILE_FLAGS=-std=c++14 -I /usr/include/eigen3

# Source files
IMU_SRC=src/imu.cpp
GPS_SRC=src/gps.cpp
UBX_SRC=src/ubx_msg.cpp
EKF_SRC=src/ekfNavINS.cpp

# Object files
IMU_OBJ=$(OBJ_DIR)/imu.o
GPS_OBJ=$(OBJ_DIR)/gps.o
UBX_OBJ=$(OBJ_DIR)/ubx_msg.o
EKF_OBJ=$(OBJ_DIR)/ekfNavINS.o

all: imu_test gps_test kalman_test graphing gps_map_test

# Pattern rule for object files
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Specific rule for cross-compiled object
$(EKF_OBJ): $(EKF_SRC)
	@mkdir -p $(@D)
	$(CROSS_COMPILE_CXX) $(CROSS_COMPILE_FLAGS) -c $< -o $@

imu_test: $(IMU_OBJ)
	$(CXX) $^ tests/imu_tests/test_imu.cpp -o test_imu $(CXXFLAGS) $(LDFLAGS)

gps_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/test_gps.cpp -o test_gps $(CXXFLAGS) $(LDFLAGS)

kalman_test: $(IMU_OBJ) $(GPS_OBJ) $(UBX_OBJ) $(EKF_OBJ)
	$(CXX) $^ tests/kalman_tests/test_kalman.cpp -o test_ekf $(CXXFLAGS) $(LDFLAGS)

graphing:
	$(CXX) -std=c++17 src/basic.cpp -o basic $(CXXFLAGS) $(LIBS)

gps_map_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/gps_map.cpp -o gps_map_test $(CXXFLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(OBJ_DIR)/*.o test_imu test_gps test_ekf basic gps_map_test
