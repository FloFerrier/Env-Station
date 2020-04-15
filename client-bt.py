# Send date and time by bluetooth to STM32
import bluetooth as bt
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

def deserializeDate(msg_date):
    return msg_date

def extractFrame(msg):
    end = msg.find(',')
    if end == -1:
        end = msg.find('\n')
    id = msg[:1]
    msg_data = msg[2:end]
    return (end + 1), id, msg_data

def deserializeMsg(msg):
    nb_data = msg.count('=')
    index = 0
    while nb_data > 0:
        i, id, msg_data = extractFrame(msg[index:])
        index += i
        nb_data -= 1
        if id == "D":
            date = deserializeDate(msg_data)
        if id == "T":
            temperature = int(msg_data)
        if id == "P":
            pressure = int(msg_data)
        if id == "H":
            humidity = int(msg_data)
        if id == "L":
            luminosity = int(msg_data) # Add log(X)
        if id == "G":
            gas = int(msg_data)
    return date, temperature, pressure, humidity, luminosity, gas

def get_msg(client_bt):
    c = 0
    msg_recv = b''
    while( c != b'\n'):
        c = client_bt.recv(1)
        msg_recv += c
    return msg_recv

def main():
    client_bt = bt.BluetoothSocket(bt.RFCOMM)
    client_bt.connect(("98:D3:61:FD:63:8F", 1))

    client_bt.send("READY\r\n")

    date_send = time.localtime()
    msg_send = serializeDate(date_send)
    #print(msg_send)
    client_bt.send(msg_send.encode('ASCII'))

    while True:
        msg_recv = get_msg(client_bt)
        date, temperature, pressure, humidity, luminosity, gas = deserializeMsg(msg_recv)
        print("Date :", date)
        print("Temperature : ", temperature)
        print("Pressure :", pressure)
        print("Humidity :", humidity)
        print("Luminosity :", luminosity)
        print("Gas :", gas)
        print("\r\n")
        client_bt.send("ACK\r\n")
    client_bt.close()

if __name__ == "__main__":
    main()
