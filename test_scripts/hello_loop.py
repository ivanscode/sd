import SocketManager as sm
import socket
import time

IP = '192.168.1.33'

def process():
    device = sm.SocketManager(IP)

    while True:
        device.send('hello')

        time.sleep(1)


def main():
    process()

if __name__ == '__main__':
    main()
