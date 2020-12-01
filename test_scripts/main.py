import SocketManager as sm
import socket
import time
import _thread as thread

DEVICE_COUNT = 3 #Not used atm
NET_BASE = '192.168.1.' #Change depending on network
RL = 4 #From IP.2
RH = 10 #To IP.20

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
    socket.setdefaulttimeout(1)
    print('Trying {}'.format(addr))
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

        if(msg == 'collect'):
            devices[0].collectData()

        elif(msg == 'demo'):
            devices[1].send('spin')
            devices[0].send('measure')            

        elif(msg == 'spin'):
            devices[0].send('spin')
            devices[1].send('spin')

        elif(msg == 'test'):
            devices[0].send('temp')
            devices[1].send('on')

        elif(msg == 'off'):
            devices[0].send('off')
            devices[1].send('off')

        elif(msg == 'on'):
            devices[0].send('on')
            devices[1].send('on')
        
        else:
            cmd = Command(msg)
            devices[cmd.id].send(cmd.msg)


def main():
    devices = findDevices()
    if len(devices) > 0:
        process(devices)          

if __name__ == '__main__': 
    main()