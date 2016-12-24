#!/usr/bin/evn python
# coding:utf-8

import time
import Queue
from vehicle import Vehicle


def send_Log(sock, ORB):
    message = ORB.dataflash()
    sock.send(message)
    # message = ORB.log_json()
    # sock.send(message)

if __name__ == '__main__':
    from apscheduler.schedulers.background import BackgroundScheduler
    scheduler = BackgroundScheduler()

    from uORB import uORB
    from library import Watcher
    mcu = None
    Watcher()
    ORB = uORB()
    print ORB._module

    # instancce of MCU module object
    if ORB.has_module('MCU'):
        from MCU_module import MCU
        mcu = MCU()

    if ORB.has_module('Compass'):
        # instancce of compass module object
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            time.sleep(.5)

    if ORB.has_module('GPS'):
        from GPS_module import GPS    # instancce of GPS module object
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            time.sleep(.5)

    if ORB.has_module('Baro'):
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            time.sleep(.5)

    if ORB.has_module('IMU'):
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.subscribe('IMU_State'):
            time.sleep(.5)

    vehicle = Vehicle(mcu, ORB)

    if ORB.has_module('Lidar'):
        from lidar_module import Lidar
        lidar = Lidar(vehicle)

    # ORB.start()

    if ORB.has_module('Cloud'):
        print('>>> Connecting to Cloud')
        from cloud_module import open_sock, Receiver, Executor

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
