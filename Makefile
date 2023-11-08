FLAGS=-li2c -ggdb

all: gps_test

imu_test:
	g++ $(FLAGS) imu_module/imu.cpp tests/imu_tests/test_imu.cpp -o test_imu 

gps_test:
	g++ $(FLAGS) gps_module/gps.cpp tests/gps_tests/test_gps.cpp -o test_gps 

clean:
	rm -rf test_imu
