#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket,threadpool,time,os,sys
from vehicle import vehicle
from config import config
from library import CancelWatcher,Watcher

global vehicle
global config
if config.get_lidar()[0] > 0:
    from lidar_module import lidar
    global lidar

pool = threadpool.ThreadPool(1)
pool2= threadpool.ThreadPool(1)
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
    if command.find('Cancel')!=-1:
        Cancel()
    elif command.find('Route')!=-1:
        info=command[6:-1]
        print 'WayPoints',info
        requests = threadpool.makeRequests(Route,(info,))
        [pool2.putRequest(req) for req in requests]             
    else:
        requests = threadpool.makeRequests(eval_wrapper,(command,))
        [pool.putRequest(req) for req in requests] 
def Route(info):
    print info
    if info == "":
        return -1
    Cancel()    # End current task
    result=[]
    wps=info.split(',')
    for wp in wps:
        loc=wp.split('+')
        result.append( [float(loc[0]),float(loc[1])] )
    # print result
    vehicle.wp=result
    vehicle.AUTO()

def Cancel():
    print "Cancel"
    CancelWatcher.Cancel=True
    time.sleep(1)
    vehicle.brake()

def eval_wrapper(command):
    """
    Execute Command from cloud
    """
    print command
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
if __name__=="__main__":
    wp="39.11111+116.33333,39.11111+116.332751132,39.1120083153+116.332751132,39.1120083153+116.333908883,39.11111+116.333908883,39.11111+116.333330015"
    vehicle.download()
    wp=vehicle.wp
    print Route(wp)
