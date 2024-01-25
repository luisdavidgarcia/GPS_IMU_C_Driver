import mmap

def read_shared_memory():
    shm_name = "/my_shared_memory"
    shm_size = 4096

    with open(shm_name, "r+b") as shm:
        mm = mmap.mmap(shm.fileno(), shm_size, access=mmap.ACCESS_READ)
        data = mm.readline().decode()
        mm.close()

    return data

sensor_data = read_shared_memory()
print(sensor_data)
