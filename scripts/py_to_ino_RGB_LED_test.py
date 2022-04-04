#!/usr/bin/env python3

# Importing Libraries
# For Arduino Communication:
# https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0

import serial
# Refer to pyserial docs: https://pyserial.readthedocs.io/en/latest/pyserial.html
# python3 -m pip install pyserial
import time
import numpy as np

# Custom Script
import image_inputs as input_image


arduino = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0.1)
# Code from: https://www.electronicshub.org/controlling-arduino-led-python/

def RGB(red,green,blue):

    arduino.write(red)
    arduino.write(green)
    arduino.write(blue)
    print('RGB should be on')


def main():    
    rgb_img = input_image.RGB
    # print('Size of RGB: ',rgb_img.size)
    # print('rgb_img[0,0] pixel 1 value:',rgb_img[0,0])
    # print('rgb_img.item(0) pixel 1 value:',rgb_img.item(0)) # gets only the 1st value
    # print('rgb_img[0,0] pixel 4 value(mid pixel):',rgb_img[1,1])
    # print('rgb_img[0,0] pixel 8 value(bottom right):',rgb_img[2,2])
    # red,green,blue = rgb_img[1,1]
    # print('Red value of pixel 4:',red)
    # print('Green value of Pixel 4:',green)
    # print('Blue value of pixel 4:',blue) 


    rows = np.array([0,1,2])
    cols = np.array([0,1,2])
    print('Number of pixels',len(rows)*len(cols))
    for i in rows:
        for j in cols:
            r,g,b = rgb_img[i,j] # stores each slice value into r,g,b (order specific!) Check in image_inputs.py to make sure you converted from BGR to RGB (thakns openCV)
            RGB(r,g,b)
    #         # if binary_img.item(i) == 255:
            #     print('Pixel Number:=',i)
            #     print('Pixel Value:=',binary_img.item(i))
            #     print('LED ON')
            #     print(arduino.readline())             #read the serial data and print it as line
            #     # time.sleep(0.5)
            #     arduino_led.led_ON()
            #     time.sleep(1.5)
            # else:
            #     print('Pixel Number:=',i)
            #     print('Pixel Value:=',binary_img.item(i))
            #     print('LED OFF')
            #     print(arduino.readline())             #read the serial data and print it as line
            #     # time.sleep(0.5)
            #     arduino_led.led_OFF()
            #     time.sleep(1.5)



    # print(arduino.readline())             #read the serial data and print it as line
    # print("RGB LED")
    # print("Enter value for Red:")
    # input_red = input()                  #waits until user enters data
    # print("Enter value for Green:")
    # input_green = input()
    # print("Enter value for Blue:")
    # input_blue = input()
    # RGB(input_red,input_green,input_blue)

if __name__ == '__main__':
    main()