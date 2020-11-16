import SocketManager as sm
import socket
import time
import threading

IP = '192.168.1.6'

class deviceThread(threading.Thread):
   def __init__(self, cmd, ip):
        threading.Thread.__init__(self)
        self.ip = ip
        self.cmd = cmd
        self.device = sm.SocketManager(IP)
   def run(self):
        print ("Starting " + self.name)
        self.device.send(self.cmd)
        print ("Exiting " + self.name)

def process():
    #thread1 = deviceThread('range', IP)
    #thread2 = deviceThread('init', '192.168.1.4')
    #thread1.start()
    #thread2.start()
    #thread1.join()
    #thread2.join()
    device1 = sm.SocketManager(IP)
    device2 = sm.SocketManager('192.168.1.4')
    device1.send('range')
    device2.send('init')
    


def main():
    process()          

if __name__ == '__main__': 
    main()