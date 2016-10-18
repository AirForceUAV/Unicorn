#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket,time,os,sys
from vehicle import vehicle
from config import config
from library import CancelWatcher,Watcher
import threading,Queue

global vehicle
global config
if config.get_lidar()[0] > 0:
    from lidar_module import lidar
    global lidar

def open_sock():
    server_address = os.path.expanduser('~') + '/.UDS'+ '_fc'
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)
        return sock
    except socket.error , msg:
        print ">>{}:{}".format(sys.stderr,msg)
        sys.exit(1)
    
class Receiver(threading.Thread):
    def __init__(self,work_queue,sock):
        threading.Thread.__init__(self,name="Receiver")
        self.work_queue=work_queue
        self.sock=sock

    def run(self):
        
        buffer_size=4096
        try:
            #use this to receive command
            while True:
                data = self.sock.recv(buffer_size)       
                if data!='':
                    vehicle.Cancel()
                    self.work_queue.put(data)            
        finally:
            print "sock is closed"
            self.sock.close()

    def _log(self,msg):
        print msg

class Executor(threading.Thread):
    def __init__(self,work_queue):
        threading.Thread.__init__(self,name="Executor")
        self.work_queue=work_queue

    def run(self):
        
        while True:
            command=self.work_queue.get()
            if command.find('Cancel')!=-1:
                self._log("Cancel")
                vehicle.Cancel()
            elif command.find('Route')!=-1:
                self._log('Route')
                info=command[7:-2]
                vehicle.Route(info)           
            else:
                self._log(command)
                eval(command)
    def _log(self,msg):
        print msg

if __name__=="__main__":
    sock=open_sock()
    work_queue=Queue.Queue()
    receiver=Receiver(work_queue,sock)
    # receiver.daemon=True
    receiver.start()

    executor=Executor(work_queue)
    # executor.daemon=True
    executor.start()

    # work_queue.join()
