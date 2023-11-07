FLAGS=-li2c

all: imu_test

imu_test:
	g++ -ggdb imu_module/imu.cpp tests/imu_tests/test_imu.cpp -o test_imu $(FLAGS)

clean:
	rm -rf test_imu
