import socket
import sys

PORT = 25565

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

class SocketManager:
    def __init__(self, ip):
        self.ip = ip

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(80)
        except socket.error as msg:
            print('Could not create socket: ' + str(msg[0]) + ': ' + msg[1])
            sys.exit(1)

        try:
            self.sock.connect((self.ip, PORT))
        except socket.error as msg:
            print('Could not open socket: ', msg)
            self.sock.close()
            sys.exit(1)

    def startThread():
        #No need for this right now
        pass

    def collectData(self):
        f = open("data.txt", "a")
        self.sock.sendall('collect'.encode())
        if(self.sock.recv(2).decode() != 'ok'):
            return
        for i in range(800):
            data = self.sock.recv(2)
            f.write("{}\n".format(int.from_bytes(data, 'little')))
            print('Distance of {} from node @ {}'.format(int.from_bytes(data, 'little'), self.ip))
        
        
        f.close()


    def send(self, cmd):
        msg = cmd.encode()
        self.sock.sendall(msg)
        data = self.sock.recv(3000)
        if not data:
            return
        print('Node @ {}: {}'.format(self.ip, data))

    
        
        