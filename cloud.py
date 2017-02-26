#!/usr/bin/evn python
# coding:utf-8

import socket
import redis
import time
import os
import sys
from library import CancelWatcher
import threading
from tools import logger


def send_Log(Redis, ORB):
    message = ORB.dataflash()
    sendChan = Redis.get('ClientSendChan').decode('utf-8')
    # print "ClientSendChan:", sendChan
    Redis.lpush(sendChan, message)
    # print 'Pushed:', message


class Receiver(threading.Thread):

    def __init__(self, work_queue, Redis):
        super(Receiver, self).__init__(name="Receiver")
        self.work_queue = work_queue
        self._redis = Redis

    def run(self):
        recvChan = self._redis.get('ClientRecvChan').decode('utf-8')
        # print "ClientRecvChan:", recvChan
        while True:
            # Using BRPOP to Receive Command
            data = self._redis.brpop(recvChan, timeout=0)
            cmd = data[1].decode('utf-8')
            if cmd is '':
                continue
            logger.debug("Received:%s" % cmd)
            if cmd.find('Cancel') != -1:
                logger.debug('Execute Cancel')
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
            if command is '':
                continue
            command = "self." + command
            logger.debug('Execute command {}'.format(command))
            try:
                eval(command)
                pass
            except Exception:
                info = sys.exc_info()
                logger.error("{0}:{1}".format(*info))
                # self.vehicle.Cancel()

if __name__ == "__main__":
    from apscheduler.schedulers.background import BackgroundScheduler
    from vehicle import Vehicle
    from library import Watcher
    from uORB import uORB
    import Queue

    ORB = uORB()
    from test_data import protobuf
    ORB._HAL = protobuf
    vehicle = Vehicle(ORB)
    scheduler = BackgroundScheduler()
    Watcher()

    Redis = redis.StrictRedis(host='localhost', port=6379, db=0)
    work_queue = Queue.Queue()

    # print '>>> Start Receiver Thread'

    receiver = Receiver(work_queue, Redis)
    receiver.daemon = True
    receiver.start()

    # print '>>> Start Executor Thread'
    executor = Executor(work_queue)
    executor.daemon = True
    executor.start()

    scheduler.add_job(send_Log, 'interval', args=(Redis, ORB), seconds=1)
    # while True:
    #     send_Log(Redis, ORB)
    #     time.sleep(1)
    scheduler.start()
    executor.join()
    receiver.join()
