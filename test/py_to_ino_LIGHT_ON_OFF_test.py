#!/usr/bin/env python3

# Importing Libraries
# For Arduino Communication:
# https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

import serial # issue with installation. Cannot find pyserial library after installing it -> FIXED
# Refer to pyserial docs: https://pyserial.readthedocs.io/en/latest/pyserial.html
# python3 -m pip install pyserial
import time

arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)


print(arduino.readline())             #read the serial data and print it as line
print("Enter 1 to ON LED and 0 to OFF LED")

while 1:                                      #infinite loop
    input_data = input()                  #waits until user enters data
    print("you entered", input_data)          #prints the data for confirmation
    
    if (input_data == '1'):                   #if the entered data is 1 
        arduino.write(b'1')            #send 1 to arduino
        print("LED ON")
       
    
    if (input_data == '0'):                   #if the entered data is 0
        arduino.write(b'0')             #send 0 to arduino (converted to bytes)
        # TypeError: unicode strings are not supported, please encode to bytes: '1' -->> convert strings to bytes
        print("LED OFF")