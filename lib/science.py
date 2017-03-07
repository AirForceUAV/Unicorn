#!/usr/bin/evn python
# coding:utf-8

import time
import math
import os
import sys


def pressure2Alt(hpa):
    # mba===hpa
    tmp = hpa / 1013.25
    return round((1 - tmp**0.190284) * 145366.45 * 0.3048, 2)


def CutFrame(package, length=2):
    pieces = [int(package[x:x + length], 16)
              for x in xrange(len(package)) if x % length == 0]
    return pieces


def CutFrame2(package, length=2):
    pieces = [package[x:x + length]
              for x in xrange(len(package)) if x % length == 0]
    return pieces


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles.
    """
    off_x = aLocation2[1] - aLocation1[1]
    off_y = aLocation2[0] - aLocation1[0]
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795 - 6
    if bearing < 0:
        bearing += 360.00
    return int(bearing)


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
    distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
    return round(distance, 2)


def angle_heading_target(origin, target, heading):
    """
    Angle from head to target (anti-clockwise)
    """
    T2NAngel = get_bearing(origin, target)
    H2TAngle = (360 + heading - T2NAngel) % 360
    return int(H2TAngle)


def angle_diff(minuend, subtrahend, sign=1):
    angle = sign * (minuend - subtrahend)
    if angle < 0:
        angle += 360
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


def dec2hex(decimal, length=4):
    """
    Convert  10h to 16h(default sizeof 2B)
    """
    hexadecimal = format(decimal, '0>' + str(length) + 'x')
    return hexadecimal


def cos(angle):
    rad = math.radians(angle)
    return round(math.cos(rad), 2)


def sin(angle):
    rad = math.radians(angle)
    return round(math.sin(rad), 2)


if __name__ == '__main__':
    loc = [36.1111, 116.2222]
    # print get_location_metres(loc, 20, 0)
    # print get_location_metres(loc, 0, 60)
    # print dec2hex(1024)
    TLocation = get_location_metres(loc, 1000, 1000)
    H2TAngle = angle_heading_target(loc, TLocation, 90)
    print H2TAngle
