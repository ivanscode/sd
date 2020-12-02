import SocketManager as sm
import socket
import time

IP = '192.168.1.4'

def process():
    device = sm.SocketManager(IP)
    
    while True:
        device.send('range')

        input('Next?')


def main():
    process()          

if __name__ == '__main__': 
    main()