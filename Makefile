FLAGS=-li2c -ggdb

all: gps_test

imu_test:
	g++ imu_module/imu.cpp tests/imu_tests/test_imu.cpp -o test_imu $(FLAGS)

gps_test:
	g++ gps_module/gps.cpp ubx_lib/ubx_msg.cpp tests/gps_tests/test_gps.cpp -o test_gps $(FLAGS)

clean:
	rm -rf test_imu test_gps
