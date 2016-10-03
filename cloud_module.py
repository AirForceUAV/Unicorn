#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket,threadpool,time,os
from vehicle import vehicle
from config import config
from library import CancelWatcher,Watcher

global vehicle
global config
if config.get_lidar()[0] > 0:
    from lidar_module import lidar
    global lidar

pool = threadpool.ThreadPool(1)
server_address = os.path.expanduser('~') + '/.UDS'+ '_fc'
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
try:
    sock.connect(server_address)
except socket.error, msg:
    print >>sys.stderr, msg
    sys.exit(1)
    

def on_message(command):
    command=command.strip()
    print str(command)
    if command.find('Cancel')==-1:
        requests = threadpool.makeRequests(eval_wrapper,(command,))
        [pool.putRequest(req) for req in requests]        
    else:
        Cancel()
        
def Cancel():
    CancelWatcher.Cancel=True
    requests = threadpool.makeRequests(eval_wrapper,('vehicle.brake()',))
    [pool.putRequest(req) for req in requests]

def eval_wrapper(command):
    """
    Execute Command from cloud
    """
    # print command
    eval(command)

def Listener():
    try:
        #use this to receive command
        while True:
            data = sock.recv(32).strip()         
            if data=='':
                continue
            else:
                # print >>sys.stderr, 'received "%s"' % data
                on_message(data)            
    finally:
        print >>sys.stderr, 'closing socket'
        sock.close()
