#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('..')
import os
import time
import socket
import threading
from lib.tools import CancelWatcher
from lib.logger import logger
import for_mc_pb2 as mc

def open_sock():
    server_address = os.path.join(os.path.expanduser('~'), '.UDS_fc')
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)
        return sock
    except socket.error as msg:
        logger.error("{}:{}".format(sys.stderr, msg))
        sys.exit(1)

def parse_ResetLoiter():
    pass

def parse_command(vehicle,lidar):      
    proto2func={mc._ResetLoiter:(vehicle.set_channels_mid,parse_ResetLoiter}

    def wrapper(cmmand_proto):
        command_object = mc.SendCommand()
        command_object.ParseFromString (command_proto)
        code = command_object.code
        assert code in proto2func,'code is error'
        func = proto2func[code]
        arg,kwargs = func[1]()
        return code2func[code]
    return wrapper

def command_args(code):

def send_Log(sock, ORB):
    message = ORB.dataflash()
    sock.send(message)


class Receiver(threading.Thread):

    def __init__(self, work_queue, sock, vehicle, lidar=None):
        super(Receiver, self).__init__(name="Receiver")
        self.work_queue = work_queue
        self.sock = sock

    def run(self):
        buffer_size = 4096
        while True:
            # use this to receive command
            cmd = self.sock.recv(buffer_size).strip()
            
            if cmd is '':
                continue
            mc_cmd = mc.SendCommand()
            mc_cmd.ParseFromString(cmd)
            if mc_cmd.code < mc._Boundary:
                logger.info('Execute Cancel')
                exc_mc_cmd(cmd,vehicle,lidar)
                # self.work_queue.put('vehicle.brake()')
            else:
                CancelWatcher.Cancel = True
                self.work_queue.put(cmd)


class Executor(threading.Thread):

    def __init__(self, work_queue, vehicle, lidar):
        super(Executor, self).__init__(name="Executor")
        self.work_queue = work_queue
        self.vehicle = vehicle
        self.lidar = lidar

    def run(self):
        command
        while True:
            command = self.work_queue.get().strip()
            if command is '':
                continue
            mc_cmd = mc.SendCommand()
            mc_cmd.ParseFromString = (command)
            code = command.code
            logger.debug('Execute command code {}'.format(code))
            try:
                eval(command)
            except Exception as e:
                info = sys.exc_info()
                logger.error("{0}:{1}".format(*info))
                # self.vehicle.Cancel()


def GCS_start(ORB, vehicle=None, lidar=None):
    from apscheduler.schedulers.background import BackgroundScheduler
    import Queue
    scheduler = BackgroundScheduler()

    sock = open_sock()
    work_queue = Queue.Queue()

    print('Start Receiver Thread')
    receiver = Receiver(work_queue, sock)
    receiver.daemon = True
    receiver.start()

    print('Start Executor Thread')
    executor = Executor(work_queue, vehicle, lidar)
    executor.daemon = True
    executor.start()

    scheduler.add_job(send_Log, 'interval', args=(sock, ORB), seconds=1)

    scheduler.start()
    # executor.join()
    # receiver.join()


if __name__ == "__main__":
    from AF_Copter.vehicle import Vehicle
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    vehicle = Vehicle(ORB)
    lidar = None

    GCS_start(ORB, vehicle, lidar)
    while True:
        time.sleep(100)
