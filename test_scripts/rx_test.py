import SocketManager as sm
import socket
import time

IP = '192.168.1.6'

def process():
    device = sm.SocketManager(IP)
    
    while True:
        device.send('rx')

        input('Next?')


def main():
    process()          

if __name__ == '__main__': 
    main()