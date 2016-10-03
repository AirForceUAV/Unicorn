#!/usr/bin/evn python
#coding:utf-8

import math,time,json
from config import config
from library import CancelWatcher,radio_package,GCS_package,list_assign
from library import get_location_metres,get_distance_metres,get_bearing,angle_heading_target
from library import Singleton

global conifg

if config.get_MCU()[0]>0:                       # instancce of MCU module object
    print 'Connecting to MCU'
    from MCU_module import mcu
    global mcu

if config.get_compass()[0]>0:
    print 'Connecting to compass'
    from compass_module import compass          # instancce of compass module object
    global compass

if config.get_GPS()[0]>0:
    print 'connecting to GPS'
    from GPS_module import gps                  # instancce of GPS module object
    global gps
    

class Vehicle(object):
    __metaclass__=Singleton
    def __init__(self):
        global config
        self._log('Vehicle Type:{}'.format(config.get_type()))
        self._log('Flight Controller:{}'.format(config.get_FC()))
        self.target= None                  # target location -- [lat,lon,alt]
        self.AIL   = config.get_AIL()      # Aileron :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.ELE   = config.get_ELE()      # Elevator:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.THR   = config.get_THR()      # Throttle:[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.RUD   = config.get_RUD()      # Rudder  :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.mode  = config.get_mode()     # Mode    :[channel number,mode1,mode2,mode3]
        self.PIT   = config.get_PIT()      # Pitch   :[channel number,low PWM ,mid PWM,high PWM ,variation PWM]
        self.gear  = config.get_gear()
        self.MD    = config.get_MD()
        self.BD    = config.get_BD()
        self.DD    = config.get_DD()
        self.mode_name='Radio'
        self.channels=self.init_channels()  # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels_mid=self.init_channels_mid()

        self.home_location=None
        self.init_alt=None
        if config.get_GPS()[0] > 0:
            self._log('Waiting for home location')
            while True:
                home=self.get_location()
                if home == None:
                    continue
                else:
                    self.home_location=home
                    break
            self._log('Home location :{}'.format(self.home_location))
            self.init_alt=gps.get_alt()                # set init altitude
            self._log('init altitude:{}'.format(self.init_alt))
        
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
        self._log('Catching Loiter PWM...')
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

    def GCS(self):
        self._log('Switch to GCS')
        self.mode_name='GCS'
        msg=GCS_package()
        mcu.send_msg(msg)

    def radio(self):
        self._log('Switch to Radio')
        self.mode_name='Radio'
        msg=radio_package()
        mcu.send_msg(msg)

    def print_channels(self):
        self._log("Current channels PWM :{}".format(self.channels))

    def print_channels_mid(self):
        self._log("Channels Mid PWM :{}".format(self.channels_mid))

    def set_gear(self,gear):
        if int(gear) in [1,2,3]:
            self.gear[0]=int(gear)-1
        else:
            self._log('Gear is unvalid')

    def get_gear(self):
        return int(self.gear[0])          

    def arm(self,duration=3):
        # self.home_location=self.get_location()
        # self.channels[self.THR[0]]=self.THR[1]
        # self.channels[self.RUD[0]]=self.RUD[3]
        # self.send_pwm()
        # time.sleep(duration)
        # self.channels[self.RUD[0]]=self.RUD[2]
        # self.send_pwm()
        pass

    def disarm(self,duration=3):
        # self.channels[self.THR[0]]=self.THR[1]
        # self.channels[self.RUD[0]]=self.RUD[1]
        # self.send_pwm()
        # time.sleep(duration)
        # self.channels[self.RUD[0]]=self.RUD[2]
        # self.send_pwm()
        pass

    def stall(self):
        self._log("Stall !!!")
        self.channels[self.THR[0]]=self.THR[1]
        self.channels[self.PIT[0]]=self.PIT[3]
        self.send_pwm()

    def takeoff(self,alt=5):
        pass
     
    
    def set_target(self,dNorth,dEast,alt=None):
        origin=self.get_location()
        if origin == None:
            self._log('GPS is Error')
            return -1
        if not str(dNorth).isdigit() or not str(dEast).isdigit():
            self._log('dNorth , dEast are unvalid')
            return -1
        if not str(alt).isdigit() or alt==None:
            self._log('Set to Current Altitude')
            alt=self.get_alt()
        self.target=get_location_metres(origin,dNorth,dEast,alt)

    def set_target2(self,lat,lon,alt=None):
        if not str(lat).isdigit() or not str(lon).isdigit():
            self._log('lat , lon are unvalid')
            return -1
        if not str(alt).isdigit() or alt==None:
            alt=self.get_alt()
        self.target=[round(lat,5),round(lon,5),alt]

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
    def movement(self,att,sign=1):
        # print att
        rate=self.gear[self.get_gear()]/100.0
        variation=int(att[5]*att[4]*rate)
        self.channels[att[0]]=att[2]+sign*variation
        # print self.channels[att[0]]

    def movement2(self,att,sign=1):
        rate=self.att[6]/100.0
        variation=int(att[5]*att[4]*rate)
        self.channels[att[0]]=att[2]+sign*variation

    def yaw_left(self):
        self.movement2(self.RUD,-1)
        self.send_pwm()
    def yaw_right(self):
        self.movement2(self.RUD)
        self.send_pwm()
    def forward(self,duration=None):
        self._log('Forward')
        self.movement(self.ELE)
        self.send_pwm()
        if str(duration).isdigit():
            time.sleep(duration)
            self.brake()

    def yaw_left_brake(self):
        duration=self.mDuration()
        self._log('Yaw Left')
        self.movement2(self.RUD,-1)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def yaw_right_brake(self):
        duration=self.mDuration()
        self._log('Yaw Right')
        self.movement2(self.RUD)
        self.yaw_send_pwm()
        time.sleep(duration)
        self.brake()
    def forward_brake(self):
        duration=self.mDuration()
        self._log('Forward')
        self.movement(self.ELE)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def backward_brake(self):
        duration=self.mDuration()
        self._log('Backward')
        self.movement(self.ELE,-1)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def roll_left_brake(self):
        duration=self.mDuration()
        self._log('Roll Left')
        self.movement(self.AIL,-1)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def roll_right_brake(self):
        duration=self.mDuration()
        self._log('Roll Right')
        self.movement(self.AIL)
        self.send_pwm()
        time.sleep(duration)
        self.brake()

    def up_brake(self):
        duration=self.mDuration()
        self._log('Throttle Up')
        self.movement(self.THR)
        self.movement(self.PIT)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def down_brake(self):
        duration=self.mDuration()
        self._log('Throttle Down')
        self.movement(self.THR,-1)
        self.movement(self.PIT,-1)    
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def mDuration(self):
        return self.MD[self.get_gear()]
    def brake(self):
        print 'brake'
        # duration=self.BD[self.get_gear()]
        # list_assign(self.channels,self.channels_mid)
        # self._log('brake')
        # self.send_pwm()
        # time.sleep(duration)

    def send_pwm(self):
        print 'send_pwm'
        # mcu.send_pwm(self.channels)

    def _angle(self,angle):
        if angle>180:
            diff=360-angle
        return angle

    def condition_yaw(self,heading=0,relative=True,deviation=2):
        """
        0<=heading<360 ;Angle deviation is degree.
        """
        if heading<0 or heading>=360:
            self._log('0<=heading<360')
            return -1
        if relative and heading==0:
            return 0

        current_heading=self.get_heading()
        watcher=CancelWatcher()
        # Relative angle to heading
        if relative:
            target_angle=(current_heading+heading)%360          
        else:
            # Absolute angle
            target_angle=heading

        direction=(360+target_angle-current_heading)%360
        if direction>0 and direction<180:
            self._log('Turn right {}'.format(direction))              
            self.yaw_right()
        elif direction>=180 and direction<360:
            self._log('Turn left {}'.format(360-direction))
            self.yaw_left()
        while not watcher.IsCancel():
            self._log('Current_angle {},{}'.format(self.get_heading(),target_angle))
            angle=(360+target_angle-self.get_heading())%360
            if self._angle(angle)<=deviation:
                break    
        self._log('Reached Angle')
        self.brake()
        return 1

    def RTL(self):
        self.guided('RTL')

    def Guided(self,_type='Guided',deviation=2):
        checktime=self.DD[self.get_gear()]
        watcher=CancelWatcher()
        if _type is "Guided":
            target=self.get_target()
            if target is None:
                self._log("Target is None!")
                self.brake()
                return -1
            self._log('Guided to Location {}'.format(target))
            
        elif _type is 'RTL':
            target=self.get_home()
            if target is None:
                self._log("Home is None!")
                self.brake()
                return -1
            self._log('RTL Location {}'.format(target))
        
        while not watcher.IsCancel():
            current_location =self.get_location()
            if current_location == None:
                self._log('GPS is Error')
                self.brake()
                return -1
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                self.brake()
                return 1    
            angle=angle_heading_target(current_location,target,self.get_heading())
            if self._angle(angle)>deviation:
                self.brake()
                self.condition_yaw(angle)
            self.forward()
            time.sleep(checktime)
        return 0

    def distance_from_home(self):
        location=self.get_location()
        home=self.get_home()
        if location is None or home is None:
            return -1
        return get_distance_metres(location,home)
    def distance_to_target(self):
        location=self.get_location()
        target=self.get_target()
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
    def FlightLog(self):
        log={}
        log["id"]=time.time()     
        if config.get_GPS()[0] > 0:
            log["HomeLocation"]=self.get_home()           # [lat,lon,alt]     
            log["LocationGlobal"]=self.get_location()     # [lat,lon,alt]
            log["DistanceFromHome"]=self.distance_from_home() # distance
            log["DistanceToTarget"]=self.distance_to_target() # distance
            log["GPS"]=gps.info()                          #[state,stars]
            log['Target']=self.get_target()                #[lat,lon,alt]
        else:
            log["HomeLocation"]=None
            log["LocationGlobal"]=None   
            log["DistanceFromHome"]=None
            log["DistanceToTarget"]=None
            log["GPS"]=None
            log['Target']=None
        if config.get_compass()[0] > 0:
            log["Gimbal"]= "{},{},{}".format(self.get_pitch(),self.get_heading(),self.get_roll())  # [pitch,yaw,roll]
            log["Compass"]=compass.info()      #[state]
        else:
            log['Gimbal']=None
            log['Compass']=None
        log["Battery"]=None       # [Voltage,Current,Capacity]
        log["Velocity"]=None      # [x,y,z]
        log["EKF"]=None
        log["Groundspeed"]=None   # speed
        log["Airspeed"]=None      # speed
        log["Mode"]=self.get_mode()  # mode
        log["IMU"]=None
        log["ServoIntput"]=None    # [ch1~ch8]
        # log["ServoInput"]=None   # [ch1~ch8]
        log["TimeStamp"]=int(time.time())
        log['Gear']=self.get_gear()+1  # Gear
        log['CurrentChannels']=','.join(self.str_channels(self.channels))    # ch1~ch8
        log['LoiterChannels']=','.join(self.str_channels(self.channels_mid)) # ch1~ch8
        log['RPM']=1600    # RPM
        
        return json.dumps(log)
    def str_channels(self,channels):
        result=[]
        for ch in channels:
            result.append(str(ch))
        return result

    def __str__(self):
        msg={}
        msg['AIL']=self.AIL
        msg['ELE']=self.ELE
        msg['THR']=self.THR
        msg['RUD']=self.RUD   
        msg['PIT']=self.PIT
        msg['Mode']=self.mode
        msg['Gear']=self.gear
        msg['Target']=self.target
        msg['Home']=self.home_location
        msg['Mode_name']=self.mode_name
        msg['Current_channels']=self.channels
        msg['Loiter_channels']=self.channels_mid
        msg['init_alt']=self.init_alt
       
        return json.dumps(msg)

    def _log(self,msg):
        print msg

vehicle=Vehicle()

if __name__=="__main__":
    # print vehicle
    # vehicle.set_channels_mid()
    # vehicle.GCS()
    # vehicle.yaw_left_brake()
    # vehicle.yaw_right_brake()
    # vehicle.roll_left_brake()
    # vehicle.roll_right_brake()
    # vehicle.forward_brake()
    # vehicle.backward_brake()
    # vehicle.up_brake()
    # vehicle.down_brake()
    # vehicle.yaw_left()
    # vehicle.yaw_right()
    # vehicle.forward(5)
    # vehicle.radio()
    vehicle.condition_yaw(30)
    vehicle.condition_yaw(270)
    
    # vehicle.set_target(40,0)
    # assert vehilce.get_location!=None,['GPS is unhealthy!!!']
    # while True:
    #     raw_input('Next')
    #     print 'heading',vehicle.get_heading()
    #     print "heading_target",angle_heading_target(vehicle.get_location(),vehicle.get_target(),vehicle.get_heading())
    # vehicle.Guided()

    


