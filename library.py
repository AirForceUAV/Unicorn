#!/usr/bin/evn python
#coding:utf-8

import time,serial,math
import signal,traceback,os,sys

def open_serial(portname,baudrate,timeout=0.5):
    com = None
    while True:
        if com is None:
          try:
            print "Connecting to port:{0},baudrate:{1}".format(portname,baudrate)
            com = serial.Serial(portname,baudrate, timeout=timeout)
            return com
          except serial.SerialException:
            info=sys.exc_info()
            print info[0],":",info[1]
            time.sleep(1.0)
            continue
def root(file_name):
        try: 
            import xml.etree.cElementTree as ET
        except ImportError: 
            import xml.etree.ElementTree as ET   
        try: 
            tree = ET.parse(file_name)
            _root = tree.getroot()
        except Exception, e: 
            print "Error:cannot parse file:",file_name
            sys.exit(1)
        return _root

def get_bearing(aLocation1, aLocation2):
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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles."""
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location[0]/180))

    # New position in decimal degrees
    newlat = original_location[0] + (dLat * 180/math.pi)
    newlon = original_location[1] + (dLon * 180/math.pi)
    targetlocation=[newlat,newlon]
            
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):  
    """
    Distance aLocation1 and aLocation2.
    """    
    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
def angle_heading_target(origin,target,heading):
    target_north=get_bearing(origin,target)
    heading_target=(360+target_north-heading)%360
    return int(heading_target)
def _angle(angle):
    if angle>180:
        return 360-angle
    else:
        return angle
def isNum(s):
    try:
        float(s)
        return True
    except ValueError:
        pass

    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass

    return False


'''
decode(解码)的作用是将其他编码的字符串转换成unicode编码, str.decode('hex')  >> hex->unicode
encode(编码)的作用是将unicode编码转换成其他编码的字符串  str.encode('hex')  >> unicode->hex
hex(int10) hex(1024) >> 0x400
int('ff',16)  >> 255
chr(0x30) >> '0'    chr(48) >> '0'
ord('0')  >> 48    '%02x'%ord('0')  >> '30'
'''    
def encode_hex(argv): 
    """
    Transform ascii to 16h
    """
    result = '' 
    hLen = len(argv)  
    for i in xrange(hLen):  
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex  
    return result

def encode_10h(int_10,length=4):
    """
    Transform  10h to 16h(sizeof 2B)
    """
    int_16=hex(int_10)[2:]
    num=len(int_16)
    if num<length:
        int_16='0'*(length-num)+int_16    
    return int_16

def list_assign(list1,list2):
    for i in xrange(len(list2)):
        list1[i]=list2[i]
def radio_package():
    return 'R'
def GCS_package():
    return 'G'
def Mid_package():
    return 'M'

class CancelWatcher(object):
    Cancel=False
    count=0
    def __init__(self):
        if self.__class__.count==0 and self.__class__.Cancel==True:
            self.__class__.Cancel=False
        self.__class__.count+=1
    def IsCancel(self):
        return self.__class__.Cancel
    def __del__(self):
        self.__class__.count-=1
        if self.__class__.count==0:
            self.__class__.Cancel = False

class Singleton(type):  
    def __init__(cls, name, bases, dict):  
        super(Singleton, cls).__init__(name, bases, dict)  
        cls._instance = None  
    def __call__(cls, *args, **kw):  
        if cls._instance is None:  
            cls._instance = super(Singleton, cls).__call__(*args, **kw)  
        return cls._instance

class Watcher(object):  
    """this class solves two problems with multithreaded 
    programs in Python, (1) a signal might be delivered 
    to any thread (which is just a malfeature) and (2) if 
    the thread that gets the signal is waiting, the signal 
    is ignored (which is a bug). 
 
    The watcher is a concurrent process (not thread) that 
    waits for a signal and the process that contains the 
    threads.  See Appendix A of The Little Book of Semaphores. 
    http://greenteapress.com/semaphores/ 
 
    I have only tested this on Linux.  I would expect it to 
    work on the Macintosh and not work on Windows. 
    """  
  
    def __init__(self):  
        """ Creates a child thread, which returns.  The parent 
            thread waits for a KeyboardInterrupt and then kills 
            the child thread. 
        """  
        self.child = os.fork()  
        if self.child == 0:  
            return  
        else:  
            self.watch()  
  
    def watch(self):  
        try:  
            os.wait()  
        except KeyboardInterrupt:  
            # I put the capital B in KeyBoardInterrupt so I can  
            # tell when the Watcher gets the SIGINT  
            print 'KeyBoardInterrupt'  
            self.kill()  
        sys.exit()  
  
    def kill(self):  
        try:  
            os.kill(self.child, signal.SIGKILL)  
        except OSError: pass  


class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False
    def __iter__(self):
        """
        Return the match method once, then stop
        """
        yield self.match
        raise StopIteration
    def match(self, *args):
        """
        Indicate whether or not to enter a case suite
        """
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

# The following example is pretty much the exact use-case of a dictionary,
# but is included for its simplicity. Note that you can include statements
# in each suite.
# v = 'ten'
# for case in switch(v):
#     if case('one'):
#         print 1
#         break
#     if case('two'):
#         print 2
#         break
#     if case('ten'):
#         print 10
#         break
#     if case('eleven'):
#         print 11
#         break
#     if case(): # default, could also just omit condition or 'if True'
#         print "something else!"
#         # No need to break here, it'll stop anyway
class Location(object):
    def __init__(self,lat,lon,alt=-1):
        self.lat=lat
        self.lon=lon
        self.alt=alt
    def __str__(self):
        print "Latitude:{} Longitude:{} Altitude:{}".format(self._lat,self._lon,self._alt)



if __name__=='__main__':
    open_serial('/dev/ttyUSB0',9600)
   

