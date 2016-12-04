#!/usr/bin/evn python
# coding:utf-8

from config import config
import time,json
from library import list_assign,get_location_metres,get_distance_metres,isNum
from waypoint import Waypoint

class Attribute(object):
    def __init__(self,mcu=None,compass=None,GPS=None,baro=None):
        self._log('Vehicle Type:{}'.format(config.get_type()))
        self._log('Flight Controller:{}'.format(config.get_FC()))
        self.mcu= mcu
        self.compass = compass
        self.gps=GPS
        self.baro=baro
        self.target= None                  # target location -- [lat,lon,alt]
        self.AIL = config.get_AIL()      # Aileron :[ch number,low PWM ,mid PWM,high PWM ,variation PWM,dir,rate]
        self.ELE = config.get_ELE()      # Elevator:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.THR = config.get_THR()      # Throttle:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.RUD = config.get_RUD()      # Rudder  :[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.mode= config.get_mode()     # Mode    :[ch number,Loiter PWM]
        self.PIT   = config.get_PIT()      # Pitch   :[chnumber, exit?]
        self.gear  = config.get_gear()
        self.MD    = config.get_MD()
        self.BD    = config.get_BD()
        self.DD    = config.get_DD()
        self.mode_name='Radio'
        self.channels=self.init_channels()  # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels_mid=self.init_channels_mid()
        self.wp=Waypoint()

        self.home_location=None
        self.init_alt=None
        if config.get_GPS()[0] > 0 and self.gps is not None:
            self._log('Waiting for home location')
            while True:
                home=self.get_location()
                stars=self.gps.get_num_stars()
                if home[2] != None:
                    self.home_location = home
                    break

            self._log('Home location :{}'.format(self.home_location))
        if config.get_Baro()[0] > 0 and self.baro is not None:
            self.init_alt = self.baro.getAlt()
            print 'init_alt is ',self.init_alt

    def download(self, index=0):
        if config.get_GPS()[0] <= 0:
            self._log("GPS is closed")
            return -1
        location=self.get_location()
        if location is None:
            self._log('GPS is not health')
            return -1
        # loc=[39.11111,116.33333]
        self.wp.download(location,index)

    def Route(self, info):
        self.wp.Route(info)

    def json_all_wp(self):
        if self.wp.all_wp() == []:
            return None
        result=[]
        for point in self.wp.all_wp():
            result.append('{}+{}'.format(point[0],point[1]))
        return ','.join(result)

    def init_channels(self):
        channels=[0,0,0,0,0,0,0,0]
        channels[self.AIL[0]]=self.AIL[2]
        channels[self.ELE[0]]=self.ELE[2]
        if self.THR[0]>0:
            channels[self.THR[0]]=self.THR[1]
        else:
            channels[self.THR[0]]=self.THR[3]
        channels[self.RUD[0]]=self.RUD[2]
        channels[self.mode[0]]=self.mode[1]
        if self.PIT[1]>0:
            channels[self.PIT[0]]=self.PIT_curve(self.THR[1])
        return channels

    def init_channels_mid(self):
        channels=[0,0,0,0,0,0,0,0]
        channels[self.AIL[0]]=self.AIL[2]
        channels[self.ELE[0]]=self.ELE[2]
        channels[self.THR[0]]=self.THR[2]
        channels[self.RUD[0]]=self.RUD[2]
        channels[self.mode[0]]=self.mode[1]
        if self.PIT[1] > 0:
            channels[self.PIT[0]]=self.PIT_curve(self.THR[2])
        return channels

    def PIT_curve(self,pwm):
        per=100*(pwm-self.THR[1])/(self.THR[3]-self.THR[1])
        PIT_PWM=int(((0.0022*per*per-0.85*per+63)/63.0)*(self.PIT[3]-self.PIT[2]))+self.PIT[2]
        return PIT_PWM

    def set_channels_mid(self):
        self._log('Catching Loiter PWM...')
        #self.mcu.send_msg('M')
        mid=self.mcu.read_channels()
        self._log('Channels Mid:{}'.format(mid))
        list_assign(self.channels,mid)
        list_assign(self.channels_mid,mid)
        self.AIL[2]=mid[self.AIL[0]]
        self.ELE[2]=mid[self.ELE[0]]
        self.THR[2]=mid[self.THR[0]]
        self.RUD[2]=mid[self.RUD[0]]

    def set_gear(self,gear):
        if int(gear) in [1,2,3]:
            self.gear[0]=int(gear)
        else:
            self._log('Gear is unvalid')

    def get_gear(self):
        return int(self.gear[0])

    def stall(self):
        self._log("stall")
        self.channels[self.THR[0]]=self.THR[1]
        if self.PIT[1]>0:
            self.channels[self.PIT[0]]=self.PIT_curve(self.THR[1])
        self.send_pwm()
         
    def set_target(self,dNorth,dEast,alt=None):
        origin=self.get_location()
        if origin == None:
            self._log('GPS is Error')
            return -1
        if not isNum(dNorth) or not isNum(dEast):
            self._log('dNorth , dEast are unvalid')
            return -1
        if alt==None:
            self._log('Set to Current Altitude')
            alt=origin[2]
        self.target=get_location_metres(origin,dNorth,dEast)

    def get_target(self):
        return self.target
    def get_heading(self):
        return self.compass.get_heading()
    def get_pitch(self):
        return self.compass.get_pitch()
    def get_roll(self):
        return self.compass.get_roll()
    def get_attitude(self):
        return self.compass.get_attitude()

    def get_mode(self):
        return self.mode_name

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
        loc=self.gps.get_location()
        return loc
    def get_alt(self):
        if self.init_alt is None:
            return None
        else:
            return self.baro.getAlt()-self.init_alt

    def FlightLog(self):
        log={}
        log["id"]=time.time()
        log["Flag"]=1     
        if config.get_GPS()[0] > 0:
            log["HomeLocation"]=self.str_list(self.get_home())           # lat,lon,alt     
            log["LocationGlobal"]=self.str_list(self.get_location())    # lat,lon,alt
            log["DistanceFromHome"]=self.distance_from_home() # distance
            log["DistanceToTarget"]=self.distance_to_target() # distance
            log["GPS"]=self.gps.info()                          #[state,stars]
            log['Target']=self.str_list(self.get_target())              #lat,lon
        else:
            log["HomeLocation"]="{},{},{}".format(36.11111,116.22222,0.0)
            log["LocationGlobal"]="{},{},{}".format(36.01234,116.12375,0.0)  
            log["DistanceFromHome"]=0.0
            log["DistanceToTarget"]=0.0
            log["GPS"]=-1
            log['Target']="{},{},{}".format(36.01234,116.12375,0.0)
        if config.get_compass()[0] > 0:
            log["Gimbal"]= self.str_list(self.get_attitude())  # [pitch,yaw,roll]
            log["Compass"]=self.compass.info()      #[state]
        else:
            log['Gimbal']="{},{},{}".format(0.2,-0.3,355)
            log['Compass']=-1
        log["Battery"]='{},{},{}'.format(12,2.5,100)       # [Voltage,Current,Capacity]
        log["Velocity"]="{},{},{}".format(0.2,0.1,0.5)      # [x,y,z]
        log["EKF"]=1
        log["Groundspeed"]=2.0   # speed
        log["Airspeed"]=3.0     # speed
        log["Mode"]=self.get_mode()  # mode
        log["IMU"]=-1
        log["TimeStamp"]=int(time.time())
        log['Gear']=self.get_gear()  # Gear
        log['CurrentChannels']=self.str_list(self.channels)   # ch1~ch8
        log['LoiterChannels']=self.str_list(self.channels_mid) # ch1~ch8
        log['CurrentWpNumber']=self.cur_wp
        log['AllWp']=self.json_all_wp()
        log['RPM']=1600    # RPM
        
        return json.dumps(log)

    def str_list(self,arrs):
        result=[]
        if arrs!=None:
            for arr in arrs:
                result.append(str(arr))
            return ','.join(result)
        else:
            return None

    def _log(self,msg):
        print msg
