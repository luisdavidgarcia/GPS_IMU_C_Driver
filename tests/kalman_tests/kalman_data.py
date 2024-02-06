import time

while True:
    try:
        with open("data.txt", "r") as file:
            data = file.read().strip()
            print(data)
    except IOError:
        print("File not accessible")
    time.sleep(1)  # Adjust the sleep time as needed
