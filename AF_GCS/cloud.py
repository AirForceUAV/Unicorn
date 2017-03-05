#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import os
import time
import redis
import threading
from lib.tools import CancelWatcher
from lib.logger import logger


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

    def __init__(self, work_queue, vehicle, lidar):
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


def cloud_start(ORB, vehicle=None, lidar=None):
    print('Initialize Cloud ...')

    from apscheduler.schedulers.background import BackgroundScheduler
    import Queue
    scheduler = BackgroundScheduler()
    r = redis.StrictRedis(host='localhost', port=6379, db=0)

    work_queue = Queue.Queue()

    print('Start Receiver Thread')
    receiver = Receiver(work_queue, r)

    receiver.daemon = True
    receiver.start()

    print('Start Executor Thread')
    executor = Executor(work_queue, vehicle, lidar)
    executor.daemon = True
    executor.start()

    scheduler.add_job(send_Log, 'interval', args=(r, ORB), seconds=1)

    scheduler.start()

if __name__ == "__main__":
    from AF_uORB.uORB import uORB
    from lib.tools import Watcher
    from AF_Copter.vehicle import Vehicle
    ORB = uORB()
    Watcher()

    from lib.proto_test import protobuf
    ORB._HAL = protobuf

    vehicle = Vehicle(ORB)
    cloud_start(ORB, vehicle)
