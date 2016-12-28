#!/usr/bin/evn python
# coding:utf-8

import time
import Queue
from vehicle import Vehicle

if __name__ == '__main__':
    from apscheduler.schedulers.background import BackgroundScheduler
    scheduler = BackgroundScheduler()

    from uORB import uORB
    from library import Watcher

    ORB = uORB()
    sbus_sender = None
    Watcher()

    if ORB.has_module('MCU'):
        # Initialize SBUS
        from sbus_receiver import Sbus_Receiver
        from sbus_sender import Sbus_Sender
        from tools import build_sbus
        com = build_sbus()
        sbus_receiver = Sbus_Receiver(ORB, com)
        sbus_receiver.start()

        while ORB.subscribe('ChannelsInput') is None:
            time.sleep(.1)

        sbus_sender = Sbus_Sender(ORB, com)
        sbus_sender.start()

    if ORB.has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            time.sleep(.1)

    if ORB.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            time.sleep(.1)

    if ORB.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            time.sleep(.1)

    if ORB.has_module('IMU'):
        # Initialize IMU
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.subscribe('IMU_State'):
            time.sleep(.1)

    # Initialize UAV
    vehicle = Vehicle(sbus_sender, ORB)

    if ORB.has_module('Lidar'):
        # Initialize Lidar
        from lidar_module import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    if ORB.has_module('Cloud'):
        # Initialize Cloud
        print('>>> Initialize Cloud ...')
        from cloud_module import open_sock, Receiver, Executor, send_Log

        sock = open_sock()
        work_queue = Queue.Queue()

        print('>>> Start Receiver Thread')
        receiver = Receiver(work_queue, sock)
        receiver.daemon = True
        receiver.start()

        print('>>> Start Executor Thread')
        executor = Executor(work_queue, vehicle)
        executor.daemon = True
        executor.start()
        from tools import protobuf
        ORB._HAL = protobuf
        scheduler.add_job(send_Log, 'interval',
                          args=(sock, ORB), seconds=1)

        # while True:
        #     message = ORB.dataflash()
        #     sock.send(message)
        #     time.sleep(1)
        scheduler.start()
        receiver.join()
        executor.join()
        work_queue.join()

    # ORB.join()
    print '>>> completed'
