import SocketManager as sm
import socket
import time

DEVICE_COUNT = 3 #Not used atm
NET_BASE = '192.168.1.' #Change depending on network
RL = 2 #From IP.2
RH = 8 #To IP.20

class Command:
    def __init__(self, data):
        self.data = data
        self.id = ''
        self.msg = ''

        self.process()

    def process(self):
        parts = self.data.split(' ')
        self.id = int(parts[0])
        self.msg = parts[1]

def scan(addr):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.setdefaulttimeout(0.5)
    result = s.connect_ex((addr, sm.PORT))
    if result == 0:
        return 1
    else :
        return 0

def findDevices():
    devices = {}
    idx = 0
    for stem in range(RL, RH):
        addr = NET_BASE + str(stem)
        if(scan(addr)):
            print('{} is available'.format(addr))
            time.sleep(0.5) #Need to wait for socket to reopen
            devices[idx] = sm.SocketManager(addr)
            idx += 1

    return devices

def process(devices):
    for i in range(0, len(devices)):
        print('Device {} available'.format(i))
    print('Usage: [device id] [command] e.g. 0 on')
    while True:
        print('==============================')
        msg = input('Enter command: ')
        assert isinstance(msg, str)

        cmd = Command(msg)
        devices[cmd.id].send(cmd.msg)


def main():
    devices = findDevices()
    if len(devices) > 0:
        process(devices)          

if __name__ == '__main__': 
    main()