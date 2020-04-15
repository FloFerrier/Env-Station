#!/usr/bin/env python3
# Script for receiving STM32 data or for updating embedded calendar
import serial
import time

def serializeDate(date):
    tmp = str(date.tm_year) + ':' + \
          str(date.tm_mon)  + ':' + \
          str(date.tm_mday) + ':' + \
          str(date.tm_wday) + ':' + \
          str(date.tm_hour) + ':' + \
          str(date.tm_min)  + ':' + \
          str(date.tm_sec)  + '\n'
    return tmp

def main():
    with serial.Serial("/dev/ttyUSB0", 115200) as ser :
        date = time.localtime()
        buffer = serializeDate(date)
        print(buffer)
        ser.write(buffer.encode('ASCII'))
        ser.close()

if __name__ == "__main__":
    main()
