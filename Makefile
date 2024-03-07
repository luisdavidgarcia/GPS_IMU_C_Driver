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
MAHONY_SRC=tests/mahony_AHRS/mahony_ahrs.cpp

# Object files
IMU_OBJ=$(OBJ_DIR)/imu.o
GPS_OBJ=$(OBJ_DIR)/gps.o
UBX_OBJ=$(OBJ_DIR)/ubx_msg.o
EKF_OBJ=$(OBJ_DIR)/ekfNavINS.o
EKF_IMU_OBJ=$(OBJ_DIR)/ekfIMU.o

all: imu_test gps_test kalman_test graphing gps_map_test

# Pattern rule for object files
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CROSS_COMPILE_FLAGS) -c $< -o $@

imu_test: $(IMU_OBJ)
	$(CXX) $^ tests/imu_tests/test_imu.cpp -o test_imu $(CXX1FLAGS) $(LDFLAGS)

imu_calibrate: $(IMU_OBJ)
	$(CXX) $^ tests/calibration/imu_mag_calibrate.cpp -o imu_calibrate $(CXX1FLAGS) $(LDFLAGS)

gps_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/test_gps.cpp -o test_gps $(CXX1FLAGS) $(LDFLAGS)

kalman_test: $(IMU_OBJ) $(GPS_OBJ) $(UBX_OBJ) $(EKF_OBJ)
	$(CXX) $^ tests/kalman_tests/test_kalman.cpp -o test_ekf $(CXX2FLAGS) $(LDFLAGS)

kalman_imu_test: $(IMU_OBJ) $(EKF_IMU_OBJ)
	$(CXX) $^ tests/kalman_tests/test_kalman_imu.cpp -o test_ekf_imu $(CXX2FLAGS) $(LDFLAGS)

mahony_AHRS: $(IMU_OBJ)
	$(CXX) $^ $(MAHONY_SRC) -o mahony_AHRS $(CXX1FLAGS) $(LDFLAGS)

graphing:
	$(CXX) -std=c++17 src/basic.cpp -o basic $(CXX1FLAGS) $(LIBS)

gps_map_test: $(GPS_OBJ) $(UBX_OBJ)
	$(CXX) $^ tests/gps_tests/gps_map.cpp -o gps_map_test $(CXX1FLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(OBJ_DIR)/*.o test_imu test_gps test_ekf basic gps_map_test
