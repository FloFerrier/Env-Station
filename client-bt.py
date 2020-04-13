# simple inquiry example
import bluetooth as bt

def main():
    module = bt.discover_devices(lookup_names=True)
    for addr, name in module:
        print("{} - {}".format(addr, name))

    client_bt = bt.BluetoothSocket(bt.RFCOMM)
    client_bt.connect(("98:D3:61:FD:63:8F", 1))
    client_bt.send("READY\r\n")

    c = 0
    msg_recv = b''
    while( c != b'\n'):
        c = client_bt.recv(1)
        msg_recv += c

    print(msg_recv)
    client_bt.send("ACK\r\n")
    client_bt.close()


if __name__ == "__main__":
    main()
