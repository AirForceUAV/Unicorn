#!/usr/bin/evn python
#coding:utf-8

import time,threading
from config import config
from vehicle import vehicle
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
    if gps.get_location() is None:
        gps.state=-1
        _log('Warning:GPS has no heartbeats')
    else:
        gps.state=1
    
def send_Log(sock,vehicle):
    message=vehicle.FlightLog()
    sock.send(message)

def test(msg):
    print msg

def _log(msg):
    print msg

if __name__=='__main__':
    global vehicle

    if config.get_lidar()[0]>0:
        from lidar_module import lidar

    timer_interval=6
    
    scheduler = BackgroundScheduler()
    # scheduler.add_job(test,'interval',args=('test',),seconds=2)

    if config.get_MCU()[0]>0:                       # instancce of MCU module object
        _log('Connecting to MCU')
        from MCU_module import mcu
        global mcu
        # scheduler.add_job(MCU_heartbeat, 'interval', args=(mcu,),seconds=timer_interval)

    if config.get_compass()[0]>0:
        _log('Connecting to compass')
        from compass_module import compass          # instancce of compass module object
        global compass
        # scheduler.add_job(Compass_heartbeat, 'interval',args=(compass,) ,seconds=timer_interval)

    if config.get_GPS()[0]>0:
        _log('Connecting to GPS')
        from GPS_module import gps                  # instancce of GPS module object
        global gps
        # scheduler.add_job(GPS_heartbeat, 'interval',args=(gps,) ,seconds=timer_interval)

    if config.get_cloud()[0]>0:
        _log('Connecting to Cloud')
        from cloud_module import sock,Listener
        Watcher()
        print 'Start Listener Thread'
        t=threading.Thread(target=Listener)
        # t.setDaemon(True)
        t.start()
        scheduler.add_job(send_Log, 'interval', args=(sock,vehicle),seconds=1)
    scheduler.start()

        