import serial
from time import sleep
import struct
import enum

speed = 0
rotation = 0

class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3
    RED = 4

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
    rotation += 1
    speed += 10


    to_pico = [255, speed, 
               254, 20, 
               253, Task.LEFT_GREEN.value] #!choose speed = float or 0-100
    ser.write(to_pico) #TODO