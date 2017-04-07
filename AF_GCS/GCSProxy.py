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


def open_sock():
    server_address = os.path.join(os.path.expanduser('~'), '.UDS_fc')
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)
        return sock
    except socket.error as msg:
        logger.error("{}:{}".format(sys.stderr, msg))
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
            cmd = self.sock.recv(buffer_size).strip()
            if not cmd or cmd is '':
                continue
            logger.info('Receive Command:{} from GCS'.format(cmd))
            if cmd.find('Cancel') != -1:
                logger.info('Execute Cancel')
                CancelWatcher.Cancel = True
                # self.work_queue.put('vehicle.brake()')
            else:
                CancelWatcher.Cancel = True
                message = str(time.time()) + '#' + cmd
                self.work_queue.put(message)


class Executor(threading.Thread):

    def __init__(self, work_queue, vehicle, lidar):
        super(Executor, self).__init__(name="Executor")
        self.work_queue = work_queue
        self.vehicle = vehicle
        self.lidar = lidar

    def run(self):
        while True:
            if self.work_queue.empty() and self.vehicle.isArmed():
                self.vehicle._brake()
                time.sleep(.01)
                continue
            message = self.work_queue.get().split('#')
            try:
                _timestamp = float(message[0])
                command = message[1].strip()
            except Exception as e:
                logger.error(e)
                continue
            timeout = time.time() - _timestamp
            if timeout > 1.5: 
                logger.debug('Timestamp is invalid timeout:{}'.format(timeout))
                continue
            if command is '':
                continue
            command = "self." + command
            logger.debug('Execute command {}'.format(command))
            try:
                eval(command)
                self.work_queue.task_done()
            except Exception as e:
                logger.error(e)
           


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
    work_queue.join()
    executor.join()
    receiver.join()


if __name__ == "__main__":
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    # Initialize UAV
    from AF_Copter.vehicle import init_vehicle
    vehicle = init_vehicle(ORB)
    lidar = None

    if config.has_module('Lidar'):
        # Initialize Lidar
        from AF_Avoid.lidar_rpc import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    GCS_start(ORB, vehicle, lidar)

    while True:
        time.sleep(10000)
