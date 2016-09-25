#!/usr/bin/evn python
#coding:utf-8

import math,time
from config import config
from library import CancelWatcher,radio_package,GCS_package,list_assign
from library import get_location_metres,get_distance_metres,get_bearing
from library import Singleton


if config.get_MCU()[0] is 1:
    print 'Connecting to MCU'
    from MCU_module import mcu

if config.get_compass()[0] is 1:
    print 'Connecting to compass'
    from compass_module import compass          # instancce of compass module object

if config.get_GPS()[0] is 1:
    print 'connecting to GPS'
    from GPS_module import gps                         # instancce of GPS module object
    assert gps!=None,["GPS is None"]
    

class Vehicle(object):
    __metaclass__=Singleton
    def __init__(self):
        global config
        self._log('Vehicle Type:{}'.format(config.get_type()))
        self._log('Flight Controller:{}'.format(config.get_FC()))
        self.mqtt=None
        self.target=None               # target location -- [lat,lon]
        self.AIL=config.get_AIL()      # Aileron :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.ELE=config.get_ELE()      # Elevator:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.THR=config.get_THR()      # Throttle:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.RUD=config.get_RUD()      # Rudder  :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.mode=config.get_mode()    # Mode    :[channel number,mode1,mode2,mode3]
        self.PIT=config.get_PIT()      # Pitch   :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.mode_name='Radio'
        self.channels=self.init_channels()  # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels_mid=self.init_channels_mid()

        self.home_location=None
        self.init_alt=None
        if config.get_GPS()[0] is 1:
            self._log('Waiting for home location')
            while True:
                home=self.get_location
                if home == None:
                    continue
                else:
                    self.home_location=home
            # self._log('Home location :{}'.format(self.home_location))
            # self.init_alt=gps.get_alt()                # set init altitude
            # self._log('init altitude:{}'.format(self.init_alt))
            pass
        
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
        self._log('Catching balance point...')
        # self.mode_name='Study'
        mid=mcu.read_channels()
        self._log('Channels Mid:{}'.format(mid))
        list_assign(self.channels,mid)
        list_assign(self.channels_mid,mid)
        self.AIL[2]=mid[self.AIL[0]]
        self.ELE[2]=mid[self.ELE[0]]
        self.THR[2]=mid[self.THR[0]]
        self.RUD[2]=mid[self.RUD[0]]
        self.PIT[2]=mid[self.PIT[0]]
        # self.radio()

    def print_channels(self):
        self._log("Current channels PWM :{}".format(self.channels))

    def print_channels_mid(self):
        self._log("Channels Mid PWM :{}".format(self.channels_mid))

    def print_func(self):
        self._log("Aileron:{} Elevator:{} Throttle:{} Rudder:{} Pitch:{} Mode:{}".format(self.AIL,self.ELE,self.THR,self.RUD,self.PIT,self.mode))

    def arm(self,duration=3):
        # self.home_location=self.get_location()
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.RUD[0]]=self.RUD[3]
        self.send_pwm()
        time.sleep(duration)
        self.channels[self.RUD[0]]=self.RUD[2]
        self.send_pwm()

    def disarm(self,duration=3):
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.RUD[0]]=self.RUD[1]
        self.send_pwm()
        time.sleep(duration)
        self.channels[self.RUD[0]]=self.RUD[2]
        self.send_pwm()

    def stall(self):
        self._log("Stall !!!")
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.PIT[0]]=self.PIT[3]
        self.send_pwm()

    # def takeoff(self,alt=5):
    #     self.channels[self.THR[0]]=self.THR[2]
    #     self.send_pwm()
    
    def set_target(self,dNorth,dEast):
        origin=self.get_location()
        if origin == None:
            return -1
        self.target=get_location_metres(origin,dNorth,dEast)

    def set_target2(self,lat,lon):
        self.target=[lat,lon]

    def get_target(self):
        return self.target

    def get_heading(self):
        return compass.get_heading()
    def get_pitch(self):
        return compass.get_pitch()
    def get_roll(self):
        return compass.get_roll()
    def get_alt(self,relative=True):
        alt=gps.get_alt()
        if relative==True:
            return alt-self.init_alt
        else:
            return alt
    def get_mode(self):
        return self.mode_name

    def set_init_alt(self):
        """
        Set init altitude
        """
        self.init_alt=gps.get_alt()

    def yaw_left(self):
        self.channels[self.RUD[0]]=self.RUD[2]-self.RUD[4]
        self.send_pwm()
    def yaw_right(self):
        self.channels[self.RUD[0]]=self.RUD[2]+self.RUD[4]
        self.send_pwm()
    def ele_forward(self):
        self._log('Forward')
        # self.channels[self.ELE[0]]=self.ELE[2]-self.ELE[4]
        # self.send_pwm()

    def turn_forward(self,heading=0,duration=3,relative=True):
        self.condition_yaw(heading,relative)
        self.ele_forward()
        time.sleep(duration)
        self.brake()

    def left(self,duration=10):
        self._log('Yaw Left')
        self.channels[self.RUD[0]]=self.RUD[2]-self.RUD[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def right(self,duration=5):
        self._log('Yaw Right')
        self.channels[self.RUD[0]]=self.RUD[2]+self.RUD[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def forward(self,duration=3):
        self._log('Forward')
        self.channels[self.ELE[0]]=self.ELE[2]-self.ELE[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def backward(self,duration=3):
        self._log('Backward')
        self.channels[self.ELE[0]]=self.ELE[2]+self.ELE[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def roll_left(self,duration=3):
        self._log('Roll Left')
        self.channels[self.AIL[0]]=self.AIL[2]+self.AIL[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def roll_right(self,duration=3):
        self._log('Roll Right')
        self.channels[self.AIL[0]]=self.AIL[2]-self.AIL[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()

    def up(self,duration=1):
        self._log('Throttle Up')
        self.channels[self.THR[0]]=self.THR[2]+self.THR[4]
        self.channels[self.PIT[0]]=self.PIT[2]-self.PIT[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def down(self,duration=1):
        self._log('Throttle Down')
        self.channels[self.THR[0]]=self.THR[2]-self.THR[4]
        self.channels[self.PIT[0]]=self.PIT[2]+self.PIT[4]
        self.send_pwm()
        time.sleep(duration)
        self.brake()

    def brake(self,duration=1):
        list_assign(self.channels,self.channels_mid)
        self._log('brake')
        self.send_pwm()
        time.sleep(duration)

    def send_pwm(self):
        return mcu.send_pwm(self.channels)

    def GCS(self):
        self._log('Switch to GCS')
        self.mode_name='PosHold'
        msg=GCS_package()
        mcu.send_msg(msg)

    def radio(self):
        self._log('Switch to Radio')
        self.mode_name='Radio'
        msg=radio_package()
        mcu.send_msg(msg)

    def _angle(self,angle):
        if angle>180:
            diff=360-angle
        return angle

    def condition_yaw(self,heading=0,relative=True,deviation=2):
        """
        0<heading<360 ;Angle deviation is degree.
        """
        if heading>=360 or (relative==True and heading==0):
            return -1
        elif heading<0 and heading>-180:
            heading+=360
        else:
            return -1

        current_heading=self.get_heading()
        watcher=CancelWatcher()
        # Relative angle to heading
        if relative==True:
            target_angle=(current_heading+heading)%360          
        else:
            # Absolute angle
            target_angle=heading

        direction=(360+target_angle-heading)%360
        if direction>0 and direction<180:
            self._log('Turn right',direction)                
            # self.yaw_right()
        elif direction>=180 and direction<360:
            self._log('Turn left',360-direction)
            # self.yaw_left()
        
        # self._log ('Need to reached angle {}'.format(target_angle))
        while not watcher.IsCancel():
            # self._log('current angle:{}'.format(self.get_heading()))
            angle=(360+target_angle-self.get_heading())
            if self._angle(angle)<=deviation:
                break    
        self._log('Reached Angle')
        # self.brake()
        return 1

    def angle_heading_target(self,origin,target):
        assert origin!=None,['Origin Location is None']
        assert target!=None,['Target Location is None']
        target_north=get_bearing(origin,target)
        heading_target=target_north-self.get_heading()
        if heading_target<0:
            heading_target+=360
        return int(heading_target)

    def RTL(self):
        self.guided('RTL')

    def Guided(self,_type='Guided',checktime=5,deviation=2):
        watcher=CancelWatcher()
        if _type is "Guided":
            target=self.get_target()
            if target is None:
                self._log("Target is None!")
                return -1
            self.mode_name='Guided'
            self._log('Guided to Location {}'.format(target))
            
        elif _type is 'RTL':
            target=self.get_home()
            if target is None:
                self._log("Home is None!")
                return -1
            self.mode_name='RTL'
            self._log('RTL Location {}'.format(target))
        elif _type is 'AUTO':
            pass

        while not watcher.IsCancel():
            current_location =self.get_location()
            if current_location == None:
                # self.cancel()
                return -1
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                # self.brake()
                return 1    
            angle=self.angle_heading_target(current_location,target)
            if self._angle(angle)>deviation:
                # self.brake()
                self.condition_yaw(angle)
            self.ele_forward()
            time.sleep(checktime)
        self.mode_name='PosHold'
        return 0

    def distance_to_home(self):
        location=self.get_location()
        home=self.get_home()
        if location is None or home is None:
            return -1
        return get_distance_metres(location,home)
    def distance_to_target(self,target):
        location=self.get_location()
        if location is None or target is None:
            return -1
        return get_distance_metres(location,target)

    def set_home(self):
        self.home_location=self.get_location()

    def get_home(self):
        return self.home_location

    def get_location(self):
        loc=gps.get_location()
        if loc is None:
            self._log("GPS is not healthy.[Debug]:num_stars is {}".format(gps.get_num_stars()))
        return loc

    def cancel(self):
        CancelWatcher.cancel=True
        self.brake()
        self.mode_name='PosHold'

    def _log(self,msg):
        print msg
        pass

vehicle=Vehicle()

if __name__=="__main__":
    # vehicle.print_channels()
    # vehicle.print_channels_mid()
    # vehicle.print_func()

    
    # vehicle.arm()
    vehicle.set_channels_mid()
    vehicle.GCS()
    # vehicle.left()
    # vehicle.right()
    vehicle.roll_left(6)
    # vehicle.roll_right(5)
    # vehicle.forward()
    # vehicle.backward()
    # vehicle.up()
    # vehicle.down()
    vehicle.radio()
    
    # vehicle.set_target(40,0)
    # assert vehilce.get_location!=None,['GPS is unhealthy!!!']
    # while True:
    #     raw_input('Next')
    #     print 'heading',vehicle.get_heading()
    #     print "heading_target",vehicle.angle_heading_target(vehicle.get_location(),vehicle.get_target())
    # vehicle.Guided()

    


