import serial
import time

# Set up the serial connection (adjust '/dev/ttyS0' and baud rate as needed)
uart_port = '/dev/ttyAMA1'
baud_rate = 115200

# Create a serial object
ser = serial.Serial(uart_port, baud_rate)

try:
    while True:
        message = "Hello UART\n"
        ser.write(message.encode('utf-8'))
        print(f"Sent: {message}")
        time.sleep(1)  # Send a message every 1 second
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()  # Ensure the serial connection is closed on exit
