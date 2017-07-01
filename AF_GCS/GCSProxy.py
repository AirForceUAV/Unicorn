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
            if not cmd:
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
        self.last_command_timestamp = 0
        # self.end_time = 0
        self.last_command = None

    def run(self):
        max_empty_queue_time = 1

        while True:
            isEmpty = self.work_queue.empty()
            if not isEmpty:
                message = self.work_queue.get()
                command = self.parseMessage(message)
                if command is None:
                    continue
 
                result = self.excute(command)
                if result:
                    self.last_command_timestamp = time.time()              
                    self.last_command = command if command.find("semi_auto")>=0 else None
                self.work_queue.task_done()
            elif isEmpty and self.vehicle.isArmed():
                empty_queue_time = time.time() - self.last_command_timestamp
                if empty_queue_time >= max_empty_queue_time:
                    logger.debug("Brake")
                    self.vehicle.brake(braketime=0.2)
                else:
                    # request to avoid_module again!
                    self.excute(self.last_command)               
            time.sleep(.01)


    def excute(self,command):
        if command is None:
            return False
        try:
            logger.debug('Execute command {}'.format(command))
            eval('self.' + command)
            return True
        except Exception as e:
            logger.error(e)
            return False
        
    def parseMessage(self,message):
        command_timeout_span = 1
        try:
            _message = message.split("#")
            timestamp = float(_message[0])
            command = _message[1].strip()
            if not command:
                logger.debug('Command is None')
                return None

            if time.time() - timestamp > command_timeout_span: 
                logger.debug('Command is timeout')
                return None
            return command
        except Exception as e:
            logger.error(e)
            return None


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
