import serial
from time import sleep
import struct

speed = 3.14159

ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate
while True:
    # received_data = ser.read()              #read serial port
    # sleep(0.03)
    # data_left = ser.inWaiting()             #check for remaining byte
    # received_data += ser.read(data_left)
    # print (received_data)                   #print received data
    # ser.write(received_data) 

    # ser.write([1])
    sleep(1)

    to_pico = [255, speed] #!choose speed = float or 0-100
    ser.write(to_pico) #TODO