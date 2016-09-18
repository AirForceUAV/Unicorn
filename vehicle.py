#!/usr/bin/evn python
#coding:utf-8

import math,time
from config import config
from GPS_module import GPS
from compass_module import compass
from mavlink_module import mavutil
import library as lib

class Drone(object):
    def __init__(self):     
        self.home_location=[]               # home location -- [lat,lon]
        self.target=[]                      # target location -- [lat,lon]

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
        channels[self.PIT[0]]=self.PIT[1]
        return channels

    def set_channels_mid(self):
        self.channels_mid=self.mav.read_channels()
        self.channels=self.mav.read_channels()

    def print_channels(self):
        self._log(self.channels)
    def print_channels_mid(self):
        self._log(self.channels_mid)

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
        time.sleep(3)
        self.channels[self.RUD[0]]=self.RUD[2]
        self.send_mavlink()

    def takeoff(self,alt=5):
        self.channels[self.THR[0]]=self.THR[2]
        self.send_mavlink()
    
    def set_target(self,dNorth,dEast):
        currentLocation=self.get_location()
        self.target=lib.get_location_metres(currentLocation,dNorth,dEast)

    def set_target2(self,lat,lon):
        self.target=(lat,lon)

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
        0<heading<360 ;angle deviation is 2 degree
        """
        watcher=CancelWatcher()
        if relative==True:
            angle=(self.get_heading()+heading)%360
        else:
            angle=heading
        if angle>180:
            angle=360-angle
            while abs(angle-self.get_heading())>2 and not wathcer.IsCancel():
                self.left()
        elif angle<180 and angle>0:
            while abs(angle-self.get_heading())>2 and not wathcer.IsCancel():
                self.right()
        self.brake()

    def simple_forward(self,heading=0,duration=3):
        self.condition_yaw(heading)
        wathcer=CancelWatcher()
        for x in range(0,int(duration)):
            self.forward()
            time.sleep(1)   
            if watcher.IsCancel() is True:
                break      
        self.brake()

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
        time.sleep(1)
        self.brake()
    def right(self):
        self.channels[self.RUD[0]]=self.RUD[2]+self.RUD[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def forward(self):
        self.channels[self.ELE[0]]=self.ELE[2]-self.ELE[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def backward(self):
        self.channels[self.ELE[0]]=self.ELE[2]+self.ELE[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def roll_left(self):
        self.channels[self.AIL[0]]=self.AIL[2]-self.AIL[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def roll_right(self):
        self.channels[self.AIL[0]]=self.AIL[2]+self.AIL[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def up(self):
        self.channels[self.THR[0]]=self.THR[2]+self.THR[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()
    def down(self):
        self.channels[self.THR[0]]=self.THR[2]-self.THR[4]
        self.send_mavlink()
        time.sleep(1)
        self.brake()

    def brake(self):
        for x in range(8):
            self.channels[x]=self.channels_mid[x]
        self.send_mavlink()

    def switch_on(self):
        msg='AABBCC'
        for i in xrange(8):
            msg+='FFF'
        msg+='DD'
        print msg
        self.mav.send_msg(msg)
    def switch_off(self):
        msg='AABBCC'
        for i in xrange(8):
            msg+='000'
        msg+='DD'
        print msg
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
    def send_mavlink(self):
        return self.mav.send_pwm(self.channels)
    def get_bearing(self,aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the 
        earth's poles.
        """ 
        off_x = aLocation2[1] - aLocation1[1]
        off_y = aLocation2[0] - aLocation1[0]
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def angle_north_target(self,target):
        '''Return the bearing between currentLocation and Target''' 
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
        self.home_location=self.GPS.get_location()

    def get_home(self):
        return self.home_location

    def get_location_metres(self,original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles."""
        earth_radius = 6378137.0  # Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location[0]/180))

        #New position in decimal degrees
        newlat = original_location[0] + (dLat * 180/math.pi)
        newlon = original_location[1] + (dLon * 180/math.pi)
        targetlocation=[newlat,newlon]
            
        return targetlocation

    def get_distance_metres(self,aLocation1, aLocation2):  
        """
        Distance aLocation1 and aLocation2.aLocation1 and aLocation are {lat:'',lon:''}
        """    
        dlat = aLocation2[0] - aLocation1[0]
        dlong = aLocation2[1] - aLocation1[1]
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    def _log(self,msg):
        print msg
        pass

class CancelWatcher(object):
    Cancel=False
    count=0
    def __init__(self):
        self.__class__.count+=1
    def IsCancel(self):
        return self.__class__.Cancel
    def __del__(self):
        self.__class__.count-=1
        if self.__class__.count==0:
            self.__class__.Cancel = False
    
if __name__=="__main__":
    drone=Drone()
    # drone.print_channels()
    # drone.set_channels_mid()
    # print drone.channels_mid
    # drone.switch_on()
    print "left"
    drone.left()
    print "right"
    drone.right()
    print "roll_left"
    drone.roll_left()
    print "roll_right"
    drone.roll_right()
    print "forward"
    drone.forward()
    print "backward"
    drone.backward()
    print "up"
    drone.up()
    print "down"
    drone.down()
    # print "disarm"
    # drone.disarm()

    # drone.set_target(50,0)
    # print drone.get_target()


