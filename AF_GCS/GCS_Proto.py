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
import protobuf.for_mc_pb2 as mc
from Parse_Proto import parse_proto, deserialize


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

    def __init__(self, work_queue, sock, parser):
        super(Receiver, self).__init__(name="Receiver")
        self.timeout = 2
        self.work_queue = work_queue
        self.sock = sock
        self.parser = parser

    def run(self):
        logger.info('Start Receiver Thread')
        concurrent_command = [mc._SetGear]
        buffer_size = 4096
        while True:
            # use this to receive command
            message = self.sock.recv(buffer_size).strip()

            if not message or message is '':
                continue
            try:
                cmd_object = deserialize(message)
                logger.debug('Receive Command:\n{}'.format(cmd_object))
            except Exception as e:
                logger.error(e)
                continue
            time_stamp = cmd_object.timestamp
            timeout = time.time() - time_stamp
            if timeout > self.timeout:
                logger.warn(
                    'Command is out-of-date,timeout:{}'.format(timeout))
                continue
            code = cmd_object.code
            if code in concurrent_command:
                self.work_queue.put(cmd_object)
            else:
                CancelWatcher.Cancel = True
                time.sleep(.1)
                try:
                    self.parser(cmd_object)
                except Exception as e:
                    logger.error(e)


class Executor(threading.Thread):

    def __init__(self, work_queue, parser):
        super(Executor, self).__init__(name="Executor")
        self.work_queue = work_queue
        self.vehicle = vehicle
        self.lidar = lidar
        self.parser = parser

    def run(self):
        logger.info('Start Executor Thread')
        while True:
            command = self.work_queue.get().strip()
            if not command or command is '':
                continue
            try:
                self.parser(cmd_object)
            except Exception as e:
                logger.error(e)


def GCS_start(ORB, vehicle=None, lidar=None):
    from apscheduler.schedulers.background import BackgroundScheduler
    import Queue
    scheduler = BackgroundScheduler()

    sock = open_sock()
    work_queue = Queue.Queue()
    parser = parse_proto(vehicle, lidar)

    receiver = Receiver(work_queue, sock, parser)
    receiver.daemon = True
    receiver.start()

    executor = Executor(work_queue, parser)
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
