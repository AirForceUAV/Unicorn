#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time
import os
import sys
from library import CancelWatcher
import threading


def open_sock():
    server_address = os.path.expanduser('~') + '/.UDS' + '_fc'
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)
        return sock
    except socket.error as msg:
        print "{}:{}".format(sys.stderr, msg)
        sys.exit(1)


def send_Log(sock, ORB):
    message = ORB.dataflash()
    sock.send(message)


class Receiver(threading.Thread):

    def __init__(self, work_queue, sock):
        super(Receiver, self).__init__(name="Receiver")
        self.work_queue = work_queue
        self.sock = sock

    def run(self):
        buffer_size = 4096
        while True:
            # use this to receive command
            cmd = self.sock.recv(buffer_size)
            if cmd is '':
                continue
            if cmd.find('Cancel') != -1:
                print 'Execute Cancel'
                CancelWatcher.Cancel = True
                # self.work_queue.put('vehicle.brake()')
            else:
                CancelWatcher.Cancel = True
                self.work_queue.put(cmd)


class Executor(threading.Thread):

    def __init__(self, work_queue, vehicle=None, lidar=None):
        super(Executor, self).__init__(name="Executor")
        self.work_queue = work_queue
        self.vehicle = vehicle
        self.lidar = lidar

    def run(self):
        while True:
            command = self.work_queue.get()
            command = "self." + command
            print 'Execute command {}'.format(command)
            try:
                # eval(command)
                pass
            except Exception:
                info = sys.exc_info()
                print "{0}:{1}".format(*info)
                # self.vehicle.Cancel()

if __name__ == "__main__":
    from apscheduler.schedulers.background import BackgroundScheduler
    from vehicle import Vehicle
    from library import Watcher
    from uORB import uORB
    import Queue

    ORB = uORB()
    from tools import protobuf
    ORB._HAL = protobuf
    vehicle = Vehicle(ORB)
    scheduler = BackgroundScheduler()
    Watcher()
    sock = open_sock()
    work_queue = Queue.Queue()

    print '>>> Start Receiver Thread'
    receiver = Receiver(work_queue, sock)
    receiver.daemon = True
    receiver.start()

    print '>>> Start Executor Thread'
    executor = Executor(work_queue, vehicle)
    executor.daemon = True
    executor.start()
    from tools import protobuf
    ORB._HAL = protobuf
    scheduler.add_job(send_Log, 'interval', args=(sock, ORB), seconds=1)
    # while True:
    #     send_Log(sock, ORB)
    #     time.sleep(1)
    scheduler.start()
    executor.join()
    receiver.join()
