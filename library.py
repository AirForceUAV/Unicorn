#!/usr/bin/evn python
# coding:utf-8

import time
import serial
import math
import signal
import os
import sys


def open_serial(portname, baudrate, timeout=None):
    while True:
        try:
            print ">>> Connecting to port:{0},baudrate:{1}".format(portname, baudrate)
            com = serial.Serial(portname, baudrate, timeout=timeout)
            return com
        except serial.SerialException:
            info = sys.exc_info()
            print "{0}:{1}".format(*info)
            time.sleep(1.0)


def read_serial(_UART, size):
    msg = ''
    try:
        msg = _UART.read(size)
    except serial.SerialException:
        info = sys.exc_info()
        print "{0}:{1}".format(*info)
    finally:
        return msg


def write_serial(_UART, info):
    num = -1
    try:
        num = _UART.read(info)
    except serial.SerialException as xxx_todo_changeme:
        serial.SerialTimeoutException = xxx_todo_changeme
        info = sys.exc_info()
        print "{0}:{1}".format(*info)
    finally:
        return num


def readline_serial(_UART):
    msg = ''
    try:
        msg = _UART.readline()
    except serial.SerialException:
        info = sys.exc_info()
        print "{0}:{1}".format(*info)
    finally:
        return msg


def element(file_name):
    try:
        import xml.etree.cElementTree as ET
    except ImportError:
        import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(file_name)
        _root = tree.getroot()
    except Exception as e:
        print "Error:cannot parse file:{0}".format(file_name)
        sys.exit(1)
    return _root


def pressure2Alt(hpa):
    # mba===hpa
    tmp = hpa / 1013.25
    return round((1 - tmp**0.190284) * 145366.45 * 0.3048, 2)


def CutFrame(package, length=2):
    FrameArray = [int(package[x:x + length], 16)
                  for x in xrange(len(package)) if x % length == 0]
    return FrameArray


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
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi *
                                            original_location[0] / 180))

    # New position in decimal degrees
    newlat = original_location[0] + (dLat * 180 / math.pi)
    newlon = original_location[1] + (dLon * 180 / math.pi)
    targetlocation = [newlat, newlon]

    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    """
    Distance aLocation1 and aLocation2.
    """
    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def angle_heading_target(origin, target, heading):
    """
    Angle from head to target (anti-clockwise)
    """
    Target2North = get_bearing(origin, target)
    Heading2Target = (360 + heading - Target2North) % 360
    return int(heading_target)


def _angle(angle):
    return angle if angle < 180 else 360 - angle


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
str.decode('hex')  >> hex->unicode
str.encode('hex')  >> unicode->hex
hex(int10) hex(1024) >> 0x400
int('ff',16)  >> 255
chr(0x30) >> '0'    chr(48) >> '0'
ord('0')  >> 48    '%02x'%ord('0')  >> '30'
'''


def ascii2hex(argv):
    """
    Convert ascii to 16h
    """
    result = ''
    hLen = len(argv)
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex
    return result


def dec2hex(int_10):
    """
    Convert  10h to 16h(sizeof 2B)
    """
    int_16 = format(hex(int_10)[2:], '0>4')
    return int_16


def cos(angle):
    rad = math.radians(angle)
    return round(math.cos(rad), 2)


def sin(angle):
    rad = math.radians(angle)
    return round(math.sin(rad), 2)


class CancelWatcher(object):
    Cancel = False
    count = 0

    def __init__(self):
        if self.__class__.count == 0 and self.__class__.Cancel:
            self.__class__.Cancel = False
        self.__class__.count += 1

    def IsCancel(self):
        return self.__class__.Cancel

    def __del__(self):
        self.__class__.count -= 1
        if self.__class__.count == 0:
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
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass

if __name__ == '__main__':
    # open_serial('/dev/compass', 9600)
    loc = [36.1111, 116.2222]
    print get_location_metres(loc, 20, 0)
    print get_location_metres(loc, 0, 60)
