CXX=g++
CXX1FLAGS=-ggdb -I include/
CXX2FLAGS=-ggdb -I /usr/include/eigen3 -I include/
LDFLAGS=-li2c
LIBS=-lmatplot -lcurl
OBJ_DIR=obj

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

all: imu_test gps_test kalman_test

# Pattern rule for object files
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXX1FLAGS) -c $< -o $@

imu_test: $(IMU_OBJ)
	$(CXX) $^ tests/imu_tests/test_imu.cpp -o imu_test $(CXX1FLAGS) $(LDFLAGS)

imu_calibrate: $(IMU_OBJ)
	$(CXX) $^ tests/calibration/imu_mag_calibrate.cpp -o imu_calibrate $(CXX1FLAGS) $(LDFLAGS)

gps_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/test_gps.cpp -o gps_test $(CXX1FLAGS) $(LDFLAGS)

# Will eventually need to add eigen3 to the include path
kalman_test: $(IMU_OBJ) $(GPS_OBJ) $(UBX_OBJ) $(EKF_OBJ)
	$(CXX) $^ tests/kalman_tests/test_kalman.cpp -o kalman_test $(CXX1FLAGS) $(LDFLAGS)

gps_map_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/gps_map.cpp -o gps_map_test $(CXX1FLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(OBJ_DIR)/*.o test_imu test_gps test_ekf basic gps_map_test
