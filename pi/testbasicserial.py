import serial
from time import sleep
import struct
import enum

ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate
while True:
    sleep(1)


    to_pico = [1] #!choose speed = float or 0-100
    ser.write(to_pico) #TODO