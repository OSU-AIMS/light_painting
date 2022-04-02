#!/usr/bin/env python3

# Importing Libraries
# For Arduino Communication:
# https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

import serial
# Refer to pyserial docs: https://pyserial.readthedocs.io/en/latest/pyserial.html
# python3 -m pip install pyserial
import time


arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)
# Code from: https://www.electronicshub.org/controlling-arduino-led-python/

def RGB(red,green,blue):
    arduino.write(red)
    arduino.write(green)
    arduino.write(blue)




def main():    
    print(arduino.readline())             #read the serial data and print it as line
    print("RGB LED")
    print("Enter value for Red:")
    input_red = input()                  #waits until user enters data
    print("Enter value for Green:")
    input_green = input()
    print("Enter value for Blue:")
    input_blue = input()
    RGB(input_red,input_green,input_blue)

if __name__ == '__main__':
    main()