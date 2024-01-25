CXX=g++
CXX1FLAGS=-ggdb -I include/
CXX2FLAGS=-ggdb -I /usr/include/eigen3 -I include/
LDFLAGS=-li2c -lserialport
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
ekf_obj: $(EKF_OBJ)

$(EKF_OBJ): $(EKF_SRC)
	@mkdir -p $(@D)
	$(CROSS_COMPILE_CXX) $(CROSS_COMPILE_FLAGS) -c $< -o $@

imu_test: $(IMU_OBJ)
	$(CXX) $^ tests/imu_tests/test_imu.cpp -o test_imu $(CXX1FLAGS) $(LDFLAGS)

gps_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/test_gps.cpp -o test_gps $(CXX1FLAGS) $(LDFLAGS)

# Exclude EKF object file and hardcode since it is cross-compiled
kalman_test: $(IMU_OBJ) $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/kalman_tests/test_kalman.cpp -o test_ekf -lrt $(EKF_OBJ) $(CXX2FLAGS) $(LDFLAGS)

graphing:
	$(CXX) -std=c++17 src/basic.cpp -o basic $(CXX1FLAGS) $(LIBS)

gps_map_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/gps_map.cpp -o gps_map_test $(CXX1FLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(OBJ_DIR)/*.o test_imu test_gps test_ekf basic gps_map_test
