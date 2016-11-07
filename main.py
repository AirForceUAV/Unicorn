#!/usr/bin/evn python
#coding:utf-8

#import logging
#logging.basicConfig()
import time,Queue
from config import config
from apscheduler.schedulers.background import BackgroundScheduler
from library import Watcher

global conifg

def MCU_heartbeat(mcu):
    # mcu.state=-1
    # _log('Warning:MCU is not heartbeat')
    pass

def Compass_heartbeat(compass):
    heading=compass.get_heading()
    if heading ==None:
        compass.state=-1
        _log('Warning:Compass has no heartbeats')
    else:
        compass.state=1
    
def GPS_heartbeat(gps):
    if gps.get_location()[2] is None:
        gps.state=-1
        _log('Warning:GPS has no heartbeats')
    else:
        gps.state=1
    
def send_Log(sock,vehicle):
    message=vehicle.FlightLog()
    message="Test"
    sock.send(message)

def _log(msg):
    print msg

if __name__=='__main__':   
    scheduler = BackgroundScheduler()

    Watcher()
    mcu=None
    gps=None
    compass=None
    if config.get_MCU()[0]>0:                       # instancce of MCU module object
        from MCU_module import MCU
        mcu=MCU()

    if config.get_compass()[0]>0:
        from compass_module import Compass          # instancce of compass module object
        compass=Compass()

        compass.start()
        while compass.get_attitude()==None:
            # print compass.get_heading()
            time.sleep(.5)

    if config.get_GPS()[0]>0:
        from GPS_module import GPS                 # instancce of GPS module object
        gps=GPS()

        gps.start()
        while gps.msg==None:
            # print gps.get_num_stars()
            time.sleep(.5)

    from vehicle import Vehicle
    vehicle=Vehicle(mcu,compass,gps)

    if config.get_lidar()[0]>0:
        from lidar_module import Lidar
        lidar=Lidar(vehicle)
    
    if config.get_cloud()[0] > 0:
        _log('Connecting to Cloud')
        from cloud_module import open_sock,Receiver,Executor

        sock = open_sock()
        work_queue=Queue.Queue()

        _log('Start Receiver Thread')
        receiver=Receiver(work_queue,sock,vehicle)
        receiver.daemon=True
        receiver.start()

        _log('Start Executor Thread')
        executor=Executor(work_queue,vehicle)
        executor.daemon=True
        executor.start()
        
        scheduler.add_job(send_Log, 'interval', args=(sock,vehicle),seconds=1)
        # while True:
        #     message=vehicle.FlightLog()
        #     sock.send(message)
        #     time.sleep(1)   
    scheduler.start()
    receiver.join()
    executor.join()
    work_queue.join()

        
