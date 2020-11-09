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

        if cmd == 'temp':
            data = self.sock.recv(2)

            num = int.from_bytes(data, 'little')
            bits = bin(num)
            print(bits)
            voltage = ((num & 0xFF) - 3.3) / 173 + 3.3
            temp = (((num >> 8) & 0xFF) - 23) * 1.14 + 23

            print('temp: {} voltage: {}'.format(temp, voltage))

            for i in range(2):
                print('{}'.format((num >> (i * 8)) & 0xFF))

            print('Node @ {}: {}'.format(self.ip, self.sock.recv(4)))

        elif cmd == 'id':
            data = self.sock.recv(4)

            #hexed = data.hex()
            #real = hexed[6:8] + xhexed[4:6] + hexed[2:4] + hexed[0:2]

            print(data.hex())

            num = int.from_bytes(data, 'little')
            print(bin(num))

            for i in range(5):
                print('{}'.format((num >> (i * 8)) & 0xFF))

            print('Node @ {}: {}'.format(self.ip, self.sock.recv(2)))
            
        else:
            data = self.sock.recv(2048)
            if not data:
                return

            print('Node @ {}: {}'.format(self.ip, data))

    
        
        