# Get data sensors from STM32 platform with Bluetooth module
import signal
import time
import bluepy
import json
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from bluepy import btle

MSG_TYPE_INFOS = 0x00
MSG_TYPE_TIME = 0x01
MSG_TYPE_BME680 = 0x02
MSG_TYPE_LPS33W = 0x03

class MsgProtocolBle():
    def __init__(self, type, len, payload):
        self.type = type
        self.len = len
        self.payload = payload

    def _msg_decode(self):
        if self.type == MSG_TYPE_BME680:
            self.timestamp = time.asctime(time.gmtime(int(self.payload[0:8], 16)))
            self.temperature = int(self.payload[8:10], 16)
            self.pressure = int(self.payload[10:14], 16)
            self.humidity = int(self.payload[14:16], 16)
        elif self.type == MSG_TYPE_LPS33W:
            self.timestamp = time.asctime(time.gmtime(int(self.payload[0:8], 16)))
            self.temperature = int(self.payload[8:10], 16)
            self.pressure = int(self.payload[10:14], 16)

    def get(self):
        self._msg_decode()
        data = {}
        if self.type == MSG_TYPE_BME680:
            data["timestamp"] = self.timestamp
            data["bme680"] = {"temperature": self.temperature, "pressure": self.pressure, "humidity": self.humidity,}
        elif self.type == MSG_TYPE_LPS33W:
            data["timestamp"] = self.timestamp
            data["lps33w"] = {"temperature": self.temperature, "pressure": self.pressure,}
        else:
            print("Unknown type ...")
        return data

class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        msg = data.decode('ascii')
        self.msg_type = msg[0:2]
        self.msg_payload_len = msg[2:4]
        self.msg_payload = msg[4:]

    @property
    def type(self):
        return int(self.msg_type)

    @property
    def payload_len(self):
        return int(self.msg_payload_len)

    @property
    def payload(self):
        return self.msg_payload

class DataSensor():
    def __init__(self, input_json):
        self._bme680_temp = []
        self._bme680_pres = []
        self._bme680_hum = []
        self._lps33w_temp = []
        self._lps33w_pres = []

        for object in input_json:
            #print("Timestamp: {}".format(object["timestamp"]))
            try:
                self._bme680_temp.append(object["bme680"]["temperature"])
            except KeyError:
                pass
            try:
                self._bme680_pres.append(object["bme680"]["pressure"])
            except KeyError:
                pass
            try:
                self._bme680_hum.append(object["bme680"]["humidity"])
            except KeyError:
                pass
            try:
                self._lps33w_temp.append(object["lps33w"]["temperature"])
            except KeyError:
                pass
            try:
                self._lps33w_pres.append(object["lps33w"]["pressure"])
            except KeyError:
                pass

    def display_temperature(self):
        x1 = range(len(self._bme680_temp))
        if len(self._bme680_temp) != 0:
            plt.plot(x1, self._bme680_temp, color='blue', label='BME680')

        x2 = range(len(self._lps33w_temp))
        if len(self._lps33w_temp) != 0:
            plt.plot(x2, self._lps33w_temp, color='red', label='LPS33W')

        plt.legend(loc='best')
        plt.ylabel("Temperature (in degrees Celsius)")
        plt.show()

    def display_pressure(self):
        x1 = range(len(self._bme680_pres))
        if len(self._bme680_pres) != 0:
            plt.plot(x1, self._bme680_pres, color='blue', label='BME680')

        x2 = range(len(self._lps33w_pres))
        if len(self._lps33w_pres) != 0:
            plt.plot(x2, self._lps33w_pres, color='red', label='LPS33W')

        plt.legend(loc='best')
        plt.ylabel("Pressure (in hPa)")
        plt.show()

    def display_humidity(self):
        x = range(len(self._bme680_hum))
        if len(self._bme680_hum) != 0:
            plt.plot(x, self._bme680_hum, color='blue', label='BME680')

        plt.legend(loc='best')
        plt.ylabel("Humidity (in %)")
        plt.show()

def msg_update_rtc():
    epoch = '{:X}'.format(int(time.time()))
    payload_len = int(len(epoch) / 2)
    str = '%DATA,01{:02X}{}%'.format(payload_len, epoch)
    return str

def write_file(ble_module, args):
    list_json = []

    msg = MyDelegate()
    ble_module.setDelegate(msg)
    ble_module.writeCharacteristic(0x0053, b'\x01\x00')

    def signal_handler(sig, frame):
        ble_module.disconnect()
        with open(args.write, 'w') as json_file:
            json.dump(list_json, json_file)
        sys.exit()
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        if ble_module.waitForNotifications(10.0):
            if msg.type == MSG_TYPE_TIME:
                str = msg_update_rtc()
                print("Send => " + str)
                ble_module.writeCharacteristic(0x0055, str.encode('ascii'))
            else:
                msg_decode = MsgProtocolBle(msg.type, msg.payload_len, msg.payload)
                data = msg_decode.get()
                print("{}".format(data))
                list_json.append(data)

def read_file(read):
    with open(read, 'r') as json_file:
        datas = DataSensor(json.load(json_file))
        datas.display_temperature()
        datas.display_pressure()
        datas.display_humidity()

def main():
    parser_args = argparse.ArgumentParser()
    parser_args.add_argument('-r', '--read', help= 'read json file')
    parser_args.add_argument('-w', '--write', help= 'write json file')
    args = parser_args.parse_args()

    if args.write is not None:
        ble_module = btle.Peripheral("04:91:62:A5:07:90")
        write_file(ble_module, args)

    elif args.read is not None:
        read_file(args.read)

if __name__ == "__main__":
    main()
