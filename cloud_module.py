#!/usr/bin/evn python
#coding:utf-8

from config import config
import time,threadpool
import paho.mqtt.client as mqtt
from vehicle import vehicle 

pool = threadpool.ThreadPool(1)

if config.get_lidar()[0] is 1:
    from Lidar import lidar 

def on_connect(client, userdata, rc):
    print "Connected cloud with result code "+str(rc)
    # Subscribe Topic "Command"
    client.subscribe("Command",qos=1)

def on_message(client, userdata, msg):
    print str(msg.payload)
    requests = threadpool.makeRequests(eval_wrapper,(str(msg.payload),))
    [pool.putRequest(req) for req in requests]

def eval_wrapper(command):
    """
    Execute Command from cloud
    """
    # print command
    eval(command)

class Cloud(object):
    def __init__(self):
        # print "Connecting to Cloud!"
        con=config.get_cloud()
        self.client = mqtt.Client(client_id='companion',clean_session=True,userdata=None)
        self.client.reinitialise(client_id='companion',clean_session=True, userdata=None)
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.connect(con[1], con[2])
        self.client.loop_start()

    def publish(self,msg,topic='Command'):
        self.client.publish(topic,msg)

    def flight_log(self):
        log={}
        log["id"]=time.time()
        # log["GPS"]=               # num_stars  
        # log["Altitude"]=         
        log["Location"]=vehicle.get_location()     # lat,lon
        log["Heading"]=vehicle.get_heading()       # heading
        log["Gimbal"]= "{},{},{}".format(self.vehicle.get_pitch(),self.vehicle.get_heading(),self.vehicle.get_roll())  # pitch,yaw,roll
        
        log["Mode"]=vehicle.mode_name              # mode_name
        log["DistanceFromHome"]=vehicle.Distance_from_home() 
        log["DistanceToTarget"]=vehicle.Distance_to_target() 
        log["ServoInput"]=None   # ch1 ch2 ch3 ch4 ch5 ch6 ch7 ch8
        log["TimeStamp"]=int(time.time())
        return log
    def publish_log(self):
        while True:
            log=vehicle.flight_log()
            self.client.publish('Command',log)
            # time.sleep(1)
# Global cloud
cloud=Cloud()

if __name__=='__main__':
    msg="vehicle.print_channels()".strip()
    while True:
        cloud.publish(msg)
        time.sleep(2)
        



 