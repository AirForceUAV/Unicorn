#!/usr/bin/evn python
#coding:utf-8

import math,time
from config import config
from GPS_module import GPS
from compass_module import compass
from mavlink_module import mavutil
from library import CancelWatcher,radio_package,GCS_package,list_assign
from library import get_location_metres,get_distance_metres,get_bearing

class Drone(object):
    def __init__(self):     
        self.home_location=[]               # home location -- [lat,lon]
        self.target=[]                      # target location -- [lat,lon]
        self.mode=''
        self.mav=mavutil()                  # instance of serial object
        # self.GPS=GPS()                      # instancce of GPS module object
        # self.horizon_alt=self.GPS.get_alt() # set horizon altitude
        # self.compass=compass()            # instancce of compass module object

        self.config=config()                # instance of configuration object
        self.AIL=self.config.get_AIL()      # Aileron :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.ELE=self.config.get_ELE()      # Elevator:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.THR=self.config.get_THR()      # Throttle:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.RUD=self.config.get_RUD()      # Rudder  :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.mode=self.config.get_Mode()    # Mode    :[channel number,mode1,mode2,mode3]
        self.PIT=self.config.get_PIT()      # Pitch   :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]

        self.channels=self.init_channels()  # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels_mid=self.init_channels_mid()

    def init_channels(self):
        channels=[0,0,0,0,0,0,0,0]
        channels[self.AIL[0]]=self.AIL[2]
        channels[self.ELE[0]]=self.ELE[2]
        channels[self.THR[0]]=self.THR[1]
        channels[self.RUD[0]]=self.RUD[2]
        channels[self.mode[0]]=self.mode[1]
        channels[self.PIT[0]]=self.PIT[1]
        return channels

    def init_channels_mid(self):
        channels=[0,0,0,0,0,0,0,0]
        channels[self.AIL[0]]=self.AIL[2]
        channels[self.ELE[0]]=self.ELE[2]
        channels[self.THR[0]]=self.THR[2]
        channels[self.RUD[0]]=self.RUD[2]
        channels[self.mode[0]]=self.mode[1]
        channels[self.PIT[0]]=self.PIT[2]
        return channels

    def set_channels_mid(self):
        mid=self.mav.read_channels()
        print 'Channels Mid:',mid
        list_assign(self.channels,mid)
        list_assign(self.channels_mid,mid)
        self.AIL[2]=self.channels_mid[self.AIL[0]]
        self.ELE[2]=self.channels_mid[self.ELE[0]]
        self.THR[2]=self.channels_mid[self.THR[0]]
        self.RUD[2]=self.channels_mid[self.RUD[0]]
        self.PIT[2]=self.channels_mid[self.PIT[0]]

    def print_channels(self):
        print "Current channels PWM :",self.channels

    def print_channels_mid(self):
        print "Channels Mid PWM :",self.channels_mid

    def print_func(self):
        print "Aileron:",self.AIL
        print "Elevatro:",self.ELE
        print "Throttle:",self.THR
        print "Rudder:",self.RUD
        print "Pitch:",self.PIT
        print "Mode:",self.mode

    def arm(self):
        # self.home_location=self.get_location()
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.RUD[0]]=self.RUD[3]
        self.send_mavlink()
        time.sleep(3)
        self.channels[self.RUD[0]]=self.RUD[2]
        self.send_mavlink()

    def set_home(self,lat,lon):
        # self.home_location=[]
        pass

    def disarm(self):
        self.home_location=self.get_location()
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.RUD[0]]=self.RUD[1]
        self.send_mavlink()
        time.sleep(5)
        self.channels[self.RUD[0]]=self.RUD[2]
        self.send_mavlink()

    def takeoff(self,alt=5):
        self.channels[self.THR[0]]=self.THR[2]
        self.send_mavlink()
    
    def set_target(self,dNorth,dEast):
        currentLocation=self.get_location()
        self.target=get_location_metres(currentLocation,dNorth,dEast)

    def set_target2(self,lat,lon):
        self.target=[lat,lon]

    def get_target(self):
        return self.target

    def get_location(self):
        location=self.GPS.get_location()
        return location

    def set_horizon_alt(self):
        """
        Set horizon altitude
        """
        self.horizon_alt=self.GPS.get_alt()

    def get_alt(self):
        return self.GPS.get_alt()-self.horizon_alt

    def condition_yaw(self,heading,relative=True):
        """
        0<heading<360 ;Angle deviation is 2 degree.
        """
        if heading<=0 or heading>=360:
            return -1

        watcher=CancelWatcher()
        if relative==True:
            angle=(self.get_heading()+heading)%360
        else:
            angle=heading
        if angle>180:
            angle=360-angle
            print 'Turn Left',angle
            self.yaw_left()
            while abs(angle-self.get_heading())>2 and not watcher.IsCancel():
                time.sleep(.2)
        elif angle<=180:
            print 'Turn Right',angle
            self.yaw_right()
            while abs(angle-self.get_heading())>2 and not watcher.IsCancel():
                time.sleep(.2)
        self.brake()
        return 1

    def yaw_left(self):
        self.channels[self.RUD[0]]=self.RUD[2]-self.RUD[4]
        self.send_mavlink()
    def yaw_right(self):
        self.channels[self.RUD[0]]=self.RUD[2]+self.RUD[4]
        self.send_mavlink()
    def ele_forward(self):
        self.channels[self.ELE[0]]=self.ELE[2]-self.ELE[4]
        self.send_mavlink()

    def turn_forward(self,heading=0,duration=3):
        self.condition_yaw(heading)
        self.ele_forward()
        
    def navigation(self):
        watcher=CancelWatcher()
        target=self.get_target()
        if target is None:
            self.drone._log("Target is None!Please set_target(lat,lon) or set_target_metres(dNorth,dEast).")
            return 0    

        while not watcher.IsCancel():
            distance=round(self.get_distance_metres(self.get_location(),target),2)          
            if distance<3:
                self._log("Reached Target Waypoint!")
                self.brake()
                return 1    
            angle=self.angle_heading_target()
            decision=strategy.Decision(angle_heading_target)
            angle=decision[1]

            self.drone.fly(distance,angle)
            # self.fly(angle)
            time.sleep(defer)
        return 0

    def left(self):
        self.channels[self.RUD[0]]=self.RUD[2]-self.RUD[4]
        self.send_mavlink()
        time.sleep(10)
        self.brake()
    def right(self):
        self.channels[self.RUD[0]]=self.RUD[2]+self.RUD[4]
        self.send_mavlink()
        time.sleep(2)
        self.brake()
    def forward(self):
        self.channels[self.ELE[0]]=self.ELE[2]-self.ELE[4]
        self.send_mavlink()
        time.sleep(2)
        self.brake()
    def backward(self):
        self.channels[self.ELE[0]]=self.ELE[2]+self.ELE[4]
        self.send_mavlink()
        time.sleep(6)
        self.brake()
    def roll_left(self):
        self.channels[self.AIL[0]]=self.AIL[2]-self.AIL[4]
        self.send_mavlink()
        time.sleep(4)
        self.brake()
    def roll_right(self):
        self.channels[self.AIL[0]]=self.AIL[2]+self.AIL[4]
        self.send_mavlink()
        time.sleep(2)
        self.brake()
    def up(self):
        self.channels[self.THR[0]]=self.THR[2]+self.THR[4]
        self.send_mavlink()
        time.sleep(2)
        self.brake()
    def down(self):
        self.channels[self.THR[0]]=self.THR[2]-self.THR[4]
        self.send_mavlink()
        time.sleep(2)
        self.brake()

    def brake(self):
        list_assign(self.channels,self.channels_mid)
        print 'brake:',self.channels
        self.send_mavlink()
        # time.sleep(1)

    def send_mavlink(self):
        return self.mav.send_pwm(self.channels)

    def GCS(self):
        self._log('Switch to GCS')
        msg=GCS_package()
        self.mav.send_msg(msg)

    def radio(self):
        self._log('Switch to radio')
        msg=radio_package()
        self.mav.send_msg(msg)

    def mode1(self):
        self.channels[self.mode[0]]=self.mode[1]
        self.send_mavlink()

    def mode2(self):
        self.channels[self.mode[0]]=self.mode[2]
        self.send_mavlink()

    def mode3(self):
        self.channels[self.mode[0]]=self.mode[3]
        self.send_mavlink() 

    def angle_north_target(self,target):
        '''
        Return the bearing between currentLocation and Target
        ''' 
        currentLocation=self.get_location()     
        angle_North=self.get_bearing(currentLocation,target)     
        return int(angle_North)

    def angle_heading_target(self):       
        target=self.get_target()
        if target is None:
            self._log("Target is None! ")
            return 0
        angle_North=self.angle_north_target(target)
        angle_heading=angle_North-self.get_heading()
        if angle_heading<0:
            angle_heading+=360
        return int(angle_heading)

    def set_home(self):
        self.home_location=self.get_location()

    def get_home(self):
        return self.home_location

    def cancel(self):
        CancelWatcher.cancel=True
        self.brake()

    def _log(self,msg):
        print msg
        pass
    
if __name__=="__main__":
    drone=Drone()
    # drone.print_channels()
    # drone.print_channels_mid()
    # drone.print_func()
    
    # time.sleep(3)
    drone.set_channels_mid()
    drone.GCS()
    # drone.arm()
    
    # time.sleep(10)
    # drone.GCS()
    # time.sleep(10)
    # drone.radio()
    # print "left"
    # drone.left()
    # print "right"
    # drone.right()
    print "roll_left"
    drone.roll_left()
    # print "roll_right"
    # drone.roll_right()
    # print "forward"
    # drone.forward()
    # print "backward"
    # drone.backward()
    # print "up"
    # drone.up()
    # print "down"
    # drone.down()
    # print "disarm"
    # drone.disarm()
    drone.radio()
    # drone.set_target(50,0)
    # print drone.get_target()


