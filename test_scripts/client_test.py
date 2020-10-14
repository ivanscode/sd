from builtins import input
import socket
import sys

# -----------  Config  ----------
PORT = 25565
IP_VERSION = 'IPv4'
IPV4 = '192.168.1.4'
# -------------------------------

family_addr = socket.AF_INET
addr = (IPV4, PORT)

try:
    sock = socket.socket(family_addr, socket.SOCK_STREAM)
except socket.error as msg:
    print('Could not create socket: ' + str(msg[0]) + ': ' + msg[1])
    sys.exit(1)

try:
    sock.connect(addr)
except socket.error as msg:
    print('Could not open socket: ', msg)
    sock.close()
    sys.exit(1)

while True:
    msg = input('Enter message to send: ')
    assert isinstance(msg, str)
    msg = msg.encode()
    sock.sendall(msg)
    #data = sock.recv(1024)
    #if not data:
    #    break
    #print('Reply: ' + data.decode())
sock.close()
