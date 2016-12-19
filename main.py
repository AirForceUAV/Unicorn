#!/usr/bin/evn python
# coding:utf-8

import time
import Queue
from config import config
from vehicle import Vehicle


def send_Log(sock, ORB):
    message = ORB.dataflash()
    sock.send(message)

if __name__ == '__main__':
    from apscheduler.schedulers.background import BackgroundScheduler
    scheduler = BackgroundScheduler()

    from uORB import uORB
    from library import Watcher
    mcu = None
    Watcher()
    ORB = uORB()
    # instancce of MCU module object
    if config.get_MCU()[0] > 0:
        from MCU_module import MCU
        mcu = MCU()

    if config.get_compass()[0] > 0:
        # instancce of compass module object
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            # print compass.get_heading()
            time.sleep(.5)

    if config.get_GPS()[0] > 0:
        from GPS_module import GPS                 # instancce of GPS module object
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    if config.get_Baro()[0] > 0:
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    vehicle = Vehicle(mcu, ORB)

    if config.get_lidar()[0] > 0:
        from lidar_module import Lidar
        lidar = Lidar(vehicle)

    if config.get_cloud()[0] > 0:
        print('Connecting to Cloud')
        from cloud_module import open_sock, Receiver, Executor

        sock = open_sock()
        work_queue = Queue.Queue()

        print('Start Receiver Thread')
        receiver = Receiver(work_queue, sock)
        receiver.daemon = True
        receiver.start()

        print('Start Executor Thread')
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
