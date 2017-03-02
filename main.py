#!/usr/bin/evn python
# coding:utf-8

import time
from vehicle import Vehicle
from config import config

if __name__ == '__main__':
    from uORB import uORB
    from library import Watcher
    ORB = uORB()
    Watcher()

    if config.has_module('Sbus'):
        # Initialize SBUS
        from sbus_receiver import Sbus_Receiver
        from sbus_sender import Sbus_Sender
        from tools import build_sbus
        com = build_sbus()
        sbus_receiver = Sbus_Receiver(ORB, com)
        sbus_receiver.start()

        while not ORB.state('Sbus'):
            time.sleep(.1)

        sbus_sender = Sbus_Sender(ORB, com)
        sbus_sender.start()

        while not ORB.state('Sender'):
            time.sleep(.1)
        print('Sbus is OK')

    if config.has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.state('Compass'):
            time.sleep(.1)
        print('Compass is OK')

    if config.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.state('GPS'):
            time.sleep(.1)
        print('GPS is OK')

    if config.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.state('Baro'):
            time.sleep(.1)
        print('Baro is OK')

    if config.has_module('IMU'):
        # Initialize IMU
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.state('IMU'):
            time.sleep(.1)
        print('IMU is OK')

    # Initialize UAV
    vehicle = Vehicle(ORB)
    lidar = None

    if config.has_module('Lidar'):
        # Initialize Lidar
        from lidar import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    if config.has_module('Cloud'):
        # Initialize Cloud

        print('Initialize Cloud ...')
        import Queue
        import redis
        from apscheduler.schedulers.background import BackgroundScheduler
        scheduler = BackgroundScheduler()

        from cloud_module import Receiver, Executor, open_sock, send_Log
        # from cloud import Receiver, Executor, send_Log

        sock = open_sock()
        # Redis = redis.StrictRedis(host='localhost', port=6379, db=0)
        work_queue = Queue.Queue()

        print('Start Receiver Thread')
        receiver = Receiver(work_queue, sock)
        # receiver = Receiver(work_queue, Redis)
        receiver.daemon = True
        receiver.start()

        print('Start Executor Thread')
        executor = Executor(work_queue, vehicle, lidar)
        executor.daemon = True
        executor.start()

        scheduler.add_job(send_Log, 'interval', args=(sock, ORB), seconds=1)
        # scheduler.add_job(send_Log, 'interval', args=(Redis, ORB), seconds=1)

        # while True:
        #     message = ORB.dataflash()
        #     # print message
        #     time.sleep(1)
        scheduler.start()

    print('completed')
