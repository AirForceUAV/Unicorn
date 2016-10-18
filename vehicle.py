#!/usr/bin/evn python
#coding:utf-8

import math,time,json
from config import config
from library import CancelWatcher,radio_package,GCS_package,list_assign
from library import get_location_metres,get_distance_metres,get_bearing,angle_heading_target
from library import Singleton,isNum,_angle
from waypoint import Waypoint

global config

if config.get_MCU()[0]>0:                       # instancce of MCU module object
    from MCU_module import mcu
    global mcu

if config.get_compass()[0]>0:
    from compass_module import compass          # instancce of compass module object
    global compass

if config.get_GPS()[0]>0:
    from GPS_module import gps                  # instancce of GPS module object
    global gps
    

class Vehicle(object):
    __metaclass__=Singleton
    def __init__(self):
        global config
        self._log('Vehicle Type:{}'.format(config.get_type()))
        self._log('Flight Controller:{}'.format(config.get_FC()))
        self.target= None                  # target location -- [lat,lon,alt]
        self.AIL   = config.get_AIL()      # Aileron :[ch number,low PWM ,mid PWM,high PWM ,variation PWM,dir,rate]
        self.ELE   = config.get_ELE()      # Elevator:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.THR   = config.get_THR()      # Throttle:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.RUD   = config.get_RUD()      # Rudder  :[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.mode  = config.get_mode()     # Mode    :[ch number,Loiter PWM]
        self.PIT   = config.get_PIT()      # Pitch   :[chnumber, exit?]
        self.gear  = config.get_gear()
        self.MD    = config.get_MD()
        self.BD    = config.get_BD()
        self.DD    = config.get_DD()
        self.mode_name='Radio'
        self.channels=self.init_channels()  # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels_mid=self.init_channels_mid()
        self.wp=[]
        self.cur_wp=0

        self.home_location=None
        self.init_alt=None
        if config.get_GPS()[0] > 0:
            self._log('Waiting for home location')
            while True:
                home=self.get_location()
                stars=gps.get_num_stars()
                if home != None and stars>9:
                    self.home_location=home
                    break
            self._log('Home location :{}'.format(self.home_location))
            self.init_alt=home[2]                # set init altitude
            self._log('init altitude:{}'.format(self.init_alt))

    def download(self,index=0):
        if config.get_GPS()[0]<=0:
            self._log("GPS is closed")
            return -1
        loc=self.get_location()
        if loc is None:
            self._log('GPS is Error')
            return -1
        # loc=[39.11111,116.33333]
        self.wp=Waypoint(loc,index).get_all_wp()
        self.AUTO()
        

    def json_all_wp(self):
        if self.wp == []:
            return None        
        result=[]
        for point in self.wp:
            result.append('{}+{}'.format(point[0],point[1]))
        return ','.join(result)
    def json_location(self):
        loc=self.get_location()
        if loc==None:
            return None
        else:
            return "{},{},{}".format(loc[0],loc[1],loc[2])
    def json_home(self):
        home=self.home_location
        if home==None:
            return None
        else:
            return "{},{},{}".format(home[0],home[1],home[2])
    def json_target(self):
        target=self.get_target()
        if target==None:
            return None
        else:
            return "{},{}".format(target[0],target[1])

    def init_channels(self):
        channels=[0,0,0,0,0,0,0,0]
        channels[self.AIL[0]]=self.AIL[2]
        channels[self.ELE[0]]=self.ELE[2]
        channels[self.THR[0]]=self.THR[1]
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
        if self.PIT[1]>0:
            channels[self.PIT[0]]=self.PIT_curve(self.THR[2])
        return channels

    def PIT_curve(self,pwm):
        per=100*(pwm-self.THR[1])/(self.THR[3]-self.THR[1])
        PIT_PWM=int(((0.0022*per*per-0.85*per+63)/63.0)*(self.PIT[3]-self.PIT[2]))+self.PIT[2]
        return PIT_PWM

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
        # self.radio()

    def GCS(self):
        self._log('Switch to GCS')
        self.mode_name='LOITER'
        msg=GCS_package()
        mcu.send_msg(msg)

    def radio(self):
        self._log('Switch to Radio')
        self.mode_name='Radio'
        msg=radio_package()
        mcu.send_msg(msg)

    def print_channels(self):
        self._log("Current PWM :{}".format(self.channels))

    def print_channels_mid(self):
        self._log("Loiter PWM :{}".format(self.channels_mid))

    def set_gear(self,gear):
        if int(gear) in [1,2,3]:
            self.gear[0]=int(gear)
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
        self._log("stall")
        self.channels[self.THR[0]]=self.THR[1]
        if self.PIT[1]>0:
            self.channels[self.PIT[0]]=self.PIT_curve(self.THR[1])
        self.send_pwm()

    def takeoff(self,alt=5):
        pass
         
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

    def set_target2(self,lat,lon,alt=None):
        if not isNum(lat) or not isNum(lon):
            self._log('lat , lon are unvalid')
            return -1
        if alt==None:
            alt=self.get_alt()
        self.target=[lat,lon]

    def get_target(self):
        return self.target
    def get_heading(self):
        return compass.get_heading()
    def get_pitch(self):
        return compass.get_pitch()
    def get_roll(self):
        return compass.get_roll()
    def get_alt(self,relative=1):
        alt=gps.get_alt()
        if relative>0:
            return alt-self.init_alt
        else:
            return alt
    def get_mode(self):
        return self.mode_name

    def movement(self,att,sign=1):
        rate=self.gear[self.get_gear()]/100.0
        variation=int(att[5]*att[4]*rate)
        self.channels[att[0]]=att[2]+sign*variation
        return self.channels[att[0]]

    def movement2(self,att,sign=1):
        rate=att[6]/100.0
        variation=int(att[5]*att[4]*rate)
        self.channels[att[0]]=att[2]+sign*variation
        return self.channels[att[0]]

    def yaw_left(self):
        self._log('Turn Left')
        self.movement2(self.RUD,-1)
        self.send_pwm()
    def yaw_right(self):
        self._log('Turn Right')
        self.movement2(self.RUD)
        self.send_pwm()
    def forward(self,duration=None):
        self._log('Forward....')
        self.movement(self.ELE)
        self.send_pwm()
        if duration!=None:
            time.sleep(duration)
            self.brake()
    def brake(self):
        self._log('brake')
        duration=self.BD[self.get_gear()]
        list_assign(self.channels,self.channels_mid)       
        self.send_pwm()
        time.sleep(duration)

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
        self.send_pwm()
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
        pwm=self.movement(self.THR)
        if self.PIT[1]>0:
            self.channels[self.PIT[0]]=self.PIT_curve(pwm)
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def down_brake(self):
        duration=self.mDuration()
        self._log('Throttle Down')
        pwm=self.movement(self.THR,-1)
        if self.PIT[1]>0:
            self.channels[self.PIT[0]]=self.PIT_curve(pwm)    
        self.send_pwm()
        time.sleep(duration)
        self.brake()
    def mDuration(self):
        return self.MD[self.get_gear()]
    

    def send_pwm(self):
        mcu.send_pwm(self.channels)

    def diff_angle(self,origin,target,sign):
        diff=(360+sign*(target-origin))%360
        if diff<180 and diff>2:
            return True
        else:
            return False

    def condition_yaw(self,heading=0,relative=True):
        """
        0<=heading<360
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
            is_cw=1
            self._log('Turn right {}'.format(direction))              
            self.yaw_right()
        elif direction>=180 and direction<360:
            is_cw=-1
            self._log('Turn left {}'.format(360-direction))
            self.yaw_left()
                  
        while not watcher.IsCancel() and self.diff_angle(self.get_heading(),target_angle,is_cw):
            self._log('Cur angle:{},Target angle:{}'.format(self.get_heading(),target_angle))
            time.sleep(.1)
        self._log('Reached Angle {}'.format(self.get_heading()))
        self.brake()
        self._log('Reached Angle {}'.format(self.get_heading()))
        return 1

    def navigation(self,target):
        deviation=config.get_degree()[0]
        checktime=config.get_DD()[4]
        watcher=CancelWatcher()
        while not watcher.IsCancel():
            current_location =self.get_location()
            if current_location == None:
                self._log("GPS is Error")
                self.brake()
                break
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                self.brake()
                break   
            angle=angle_heading_target(current_location,target,self.get_heading())
            if _angle(angle)>deviation:
                self.brake()
                self.condition_yaw(angle)
            self.forward()
            time.sleep(checktime)
        return 0
    def RTL(self):
        target=self.get_home()
        if target is None:                
            self._log("Home is None!")
            self.brake()
            return -1
        self.mode_name='RTL'
        
        self.navigation(target)
        self.mode_name="Loiter"

    def Auto(self):
        if self.wp == []:
            self._log('Waypoint is none')
            return -1
        self.mode_name='AUTO'
        watcher=CancelWatcher()
        for point in self.wp:
            if watcher.IsCancel():
                break
            self.cur_wp+=1
            self._log("Target is None!")
            self.navigation(point)
        
        self.mode_name="Loiter"
        self.clear()

    def Guided(self):
        target=self.get_target()
        if target is None:                
            self._log("Target is None!")
            self.brake()
            return -1
        self.mode_name="GUIDED"
            
        self.navigation(target)
        self.mode_name="Loiter"
        self.target=None
    def Route(self,info):
        self._log(info)
        if info == "":
            return -1
        self.Cancel()    # End current task
        result=[]
        wps=info.split(',')
        for wp in wps:
            loc=wp.split('+')
            result.append( [float(loc[0]),float(loc[1])] )
        self._log(result)
        self.wp=result
        self.Auto()

    def clear(self):
        self.wp=[]
        self.cur_wp=0

    def GPS_ERR(self):
        return "GPS is ERROR ! num_stars:{}".format(gps.get_num_stars())

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
            self._log("GPS is not healthy.num_stars is {}".format(gps.get_num_stars()))
        return loc
    def FlightLog(self):
        log={}
        log["id"]=time.time()
        log["Flag"]=1     
        if config.get_GPS()[0] > 0:
            log["HomeLocation"]=self.json_home()           # lat,lon,alt     
            log["LocationGlobal"]=self.json_location()     # lat,lon,alt
            log["DistanceFromHome"]=self.distance_from_home() # distance
            log["DistanceToTarget"]=self.distance_to_target() # distance
            log["GPS"]=gps.info()                          #[state,stars]
            log['Target']=self.json_target()                #lat,lon
        else:
            log["HomeLocation"]="{},{},{}".format(36.11111,116.22222,0.0)
            log["LocationGlobal"]="{},{},{}".format(36.01234,116.12375,0.0)  
            log["DistanceFromHome"]=0.0
            log["DistanceToTarget"]=0.0
            log["GPS"]=-1
            log['Target']="{},{},{}".format(36.01234,116.12375,0.0)
        if config.get_compass()[0] > 0:
            log["Gimbal"]= "{},{},{}".format(self.get_pitch(),self.get_heading(),self.get_roll())  # [pitch,yaw,roll]
            log["Compass"]=-1      #[state]
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
        # log["ServoIntput"]=None    # [ch1~ch8]
        # log["ServoInput"]=None   # [ch1~ch8]
        log["TimeStamp"]=int(time.time())
        log['Gear']=self.get_gear()  # Gear
        log['CurrentChannels']=','.join(self.str_channels(self.channels))    # ch1~ch8
        log['LoiterChannels']=','.join(self.str_channels(self.channels_mid)) # ch1~ch8
        log['CurrentWpNumber']=self.cur_wp
        log['AllWp']=self.json_all_wp()
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
    def Cancel(self):
        self._log("Cancel")
        CancelWatcher.Cancel=True
        time.sleep(1)
        self.brake()

    def _log(self,msg):
        print msg

vehicle=Vehicle()

if __name__=="__main__":
    # vehicle.print_channels()
    # vehicle.print_channels_mid()
    
    #while True:
    #    #raw_input("NEXT")
    #    time.sleep(.5)
    #    vehicle.set_channels_mid()
    #    print vehicle.PIT_curve(vehicle.channels_mid[2])
    #vehicle.GCS()
    #vehicle.set_channels_mid()
    ##vehicle.set_gear(2)
    #vehicle.yaw_left_brake()
    # time.sleep(3)
    #vehicle.yaw_right_brake()
    # vehicle.roll_left_brake()
    # time.sleep(2)
    # vehicle.roll_right_brake()
    #vehicle.forward_brake()
    # time.sleep(2)
    # vehicle.backward_brake()
    # vehicle.up_brake()
    # time.sleep(2)
    #vehicle.down_brake()
    #vehicle.yaw_left()
    #vehicle.yaw_right()
    #vehicle.forward(5)
   
    vehicle.condition_yaw(30)
    vehicle.condition_yaw(270)
    #
    #vehicle.set_target(20,0)
    #assert vehilce.get_location!=None,['GPS is unhealthy!!!']
    # while True:
    #     raw_input('Next')
    #     print 'heading',vehicle.get_heading()
    #     print "heading_target",angle_heading_target(vehicle.get_location(),vehicle.get_target(),vehicle.get_heading())
    #vehicle.Guided()
    vehicle.radio()
    


