#!/usr/bin/python
# -*- coding=utf-8 -*-

from library import element, get_location_metres


class Waypoint(object):

    def __init__(self):
        self._wp = None
        self._number = -1

    def download(self, origin, index=0):
        file_path = 'waypoint.xml'
        _root = element(file_path)[index]
        result = [origin]
        points = _root.getchildren()
        if points is None:
            return 0
        number = 0
        for point in points:
            result.append(get_location_metres(
                result[number], self.child(point, 0), self.child(point, 1)))
            number += 1
        self._wp = result[1:]
        self._number = 0

    def Route(self, info):
        if info == "":
            print "Route is None"
            return -1
        result = []
        wps = info.split(',')
        for wp in wps:
            loc = wp.split('+')
            result.append([float(loc[0]), float(loc[1])])
        self._wp = result
        self._number = 0

    def remain_wp(self):
        return self._wp[self._number:]

    def all_wp(self):
        return self._wp

    def current_wp(self):
        return self._wp[self._number]

    def add_number(self):
        self._number += 1

    def minus_number(self):
        if self._number < 0:
            self._number = -1
        else:
            self._number -= 0

    def child(self, point, index):
        return float(point[index].text)

    def write_xml(self, out_path):
        self._root.write(out_path, encoding="utf-8", xml_declaration=True)

    def clear(self):
        self._wp = None
        self._number = -1

    def isNull(self):
        if self._wp is None:
            return True
        else:
            return False

    def _log(self, msg):
        print msg

if __name__ == "__main__":
    wp = Waypoint()
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    print wp.all_wp()
    info = "36.11111+116.22222,36.11111+116.22222,36.11111+116.22222,36.11111+116.22222,36.11111+116.22222"
    wp.Route(info)
    print wp.all_wp()
