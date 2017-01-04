#!/usr/bin/env python

import socket 
import serial
import time

TCP_IP = '192.168.1.7'
TCP_PORT = 7
BUFFER_SIZE = 1024
MESSAGE = "Hello World"
i = 0;

ser = serial.Serial('/dev/ttyUSB1', 2e6, timeout=0.1)
print(ser.name)
  
while True:
        
    # Send data over Ethernet/TCP
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE + ' ' + str(i))
    data = s.recv(BUFFER_SIZE)
    s.close()   

    # Receive data from TCP and USART
    print 'Original data        : ', MESSAGE, str(i)
    print 'Received data (TCP)  :' , repr(data)
    print 'Received data (USART): ', ser.readline()
    print

    # Change message and delay
    i += 1
    #time.sleep(1)
