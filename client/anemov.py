

import bluetooth

import time
class BTComm(object):

    def __init__(self, addr, port=1):

        self.addr = addr
        self.port = 1
        self.isopen = False
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.f = None
        
    def connect(self):
        self.sock.connect((self.addr, self.port))
        self.isopen = True
        self.sock.settimeout(3)
        
    def close(self):
        self.sock.close(self)
        self.isopen = False
    def acquire(self):
        if not self.isopen:
            self.connect()
        self.sock.send(b'?')
        buf = b''
        
        for i in range(5):
            buf = buf + self.sock.recv(1024)
            if buf.find(b'END') >= 0:
                break
        return buf

        
            
        
            
        
