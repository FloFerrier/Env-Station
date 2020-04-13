#!/usr/bin/env python3
# Script for setting HC-05 bluetooth module
import serial

def hc05_rsp(ser):
    rsp = ser.read_until(b'\n')
    print(rsp.decode("ascii").rstrip())
    if((rsp.find(b'OK') < 0) and   \
       (rsp.find(b'FAIL') < 0) and \
       (rsp.find(b'ERROR') < 0)):
        rsp = ser.read_until(b'\n')
        print(rsp.decode("ascii").rstrip())

def send_hc05_command(ser, cmd):
    print(cmd + ": ", end="")
    ser.write(bytes(cmd+"\r\n", "ascii"))
    return hc05_rsp(ser)

def main():
    with serial.Serial("/dev/ttyUSB0", 38400) as ser :
        send_hc05_command(ser, "AT")
        #send_hc05_command(ser, "AT+RESET", "OK")
        send_hc05_command(ser, "AT+ORGL")
        send_hc05_command(ser, "AT+ADDR?")
        send_hc05_command(ser, "AT+NAME?")
        send_hc05_command(ser, "AT+PSWD?")
        send_hc05_command(ser, "AT+BIND?")
        send_hc05_command(ser, "AT+NAME=Test_HC05")
        send_hc05_command(ser, "AT+BIND=BL, 00, 01")
        send_hc05_command(ser, "AT+PSWD=0501")
        send_hc05_command(ser, "AT+CMODE=1")
        send_hc05_command(ser, "AT+ROLE?")
        send_hc05_command(ser, "AT+STATE?")
        ser.close()

if __name__ == "__main__":
    main()
