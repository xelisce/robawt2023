import serial
import struct
from time import sleep, time

ser = serial.Serial ("/dev/ttyS0", 115200)

temp_var = 0
start = time() 

while True:
    received_data = ser.read()
    sleep(0.03)
    data_left = ser.inWaiting()
    received_data += ser.read(data_left)
    print(str(received_data))
    if time() - start > 2:
        ser.write(struct.pack('f', 1.0))
        temp_var += 1
        start = time()

