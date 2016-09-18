#!/usr/bin/evn python
#coding:utf-8

import time,serial,traceback,sys

def open_serial(portname,baudrate):
    com = None
    while True:
        if com is None:
          try:
            print "Connecting to port:{0},baudrate:{1}".format(portname,baudrate)
            com = serial.Serial(portname,baudrate, timeout=1.0)
            return com
          except serial.SerialException:
            # traceback.print_exc()
            info=sys.exc_info()
            print info[0],":",info[1]
            time.sleep(1.0)
            continue

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
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location[0]/180))

        #New position in decimal degrees
        newlat = original_location[0] + (dLat * 180/math.pi)
        newlon = original_location[1] + (dLon * 180/math.pi)
        targetlocation=[newlat,newlon]
            
        return targetlocation

def get_distance_metres(aLocation1, aLocation2):  
    """
    Distance aLocation1 and aLocation2.aLocation1 and aLocation are {lat:'',lon:''}
    """    
    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def encode_hex(argv): 
    """
    example:
    "hello" ->  "68 65 6c 6c 6f"
    """
    result = '' 
    hLen = len(argv)  
    for i in xrange(hLen):  
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex+' '  
    return result

def encode_10h(int_10):
    """
    Transform  10h to 16h
    """
    int_16=hex(int_10)[2:]
    length=len(int_16)
    if length==2:
        int_16='0'+int_16
    elif length==1:
        int_16='00'+int_16
    return int_16

if __name__=='__main__':
    print encode_hex('aabbcc',3)
