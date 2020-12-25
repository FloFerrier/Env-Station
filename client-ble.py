# Get data sensors from STM32 platform with Bluetooth module
import signal
import time
import bluepy
from bluepy import btle

class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        # ... perhaps check cHandle
        # ... process 'data'
        data_process(data.decode('ascii'))

def data_process(data):
    print(data)
    tmp = data.split(',', 1)
    msg = tmp[1]
    print("Type: " + msg[0:2])
    print("Len: " + msg[2:4])
    print("Payload: " + msg[4:])

def msg_update_rtc():
    epoch = '{:X}'.format(int(time.time()))
    payload_len = int(len(epoch) / 2)
    str = '%DATA,01{:02X}{}%'.format(payload_len, epoch)
    return str

def main():
    ble_module = btle.Peripheral("04:91:62:A5:07:90")
    ble_module.setDelegate(MyDelegate())
    ble_module.writeCharacteristic(0x0053, b'\x01\x00')

    def exit(signum, frame):
        ble_module.disconnect()
        exit()
    signal.signal(signal.SIGTERM, exit)

    while(1):
        if(ble_module.waitForNotifications(10.0)):
            str = msg_update_rtc()
            print("Send => " + str)
            ble_module.writeCharacteristic(0x0055, str.encode('ascii'))

if __name__ == "__main__":
    main()
