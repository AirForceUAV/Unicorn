#!/usr/bin/python
# -*- coding=utf-8 -*-

from library import element, get_location_metres


class Waypoint(object):

    def __init__(self, ORB):
        file_path = 'waypoint.xml'
        self._root = element(file_path)
        self.ORB = ORB

    def download(self, origin, index=0):
        _root = self._root[index]
        result = [origin]
        points = _root.getchildren()
        if points is None:
            return 0
        number = 0
        for point in points:
            result.append(get_location_metres(
                result[number], self.child(point, 0), self.child(point, 1)))
            number += 1
        self.ORB.publish('Waypoint', result[1:])
        self.ORB.publish('WaypointID', 0)

    def Route(self, info):
        if info == "":
            print "Route is None"
            return -1
        result = []
        wps = info.split(',')
        for wp in wps:
            loc = wp.split('+')
            result.append([float(loc[0]), float(loc[1])])
        self.ORB.publish('Waypoint', result)
        self.ORB.publish('WaypointID', 0)

    def remain_wp(self):
        ID = self.ORB.subscribe('WaypointID')
        Waypoint = self.ORB.subscribe('Waypoint')
        if Waypoint is [] or ID >= len(Waypoint):
            return []
        return Waypoint[ID:]

    def all_wp(self):
        return self.ORB.subscribe('Waypoint')

    def current_wp(self):
        return self._wp[self._number]

    def add_number(self):
        self.ORB._HAL['WaypointID'] += 1

    def child(self, point, index):
        return float(point[index].text)

    def write_xml(self, out_path):
        self._root.write(out_path, encoding="utf-8", xml_declaration=True)

    def clear(self):
        self.ORB.publish('Waypoint', [])
        self.ORB.publish('WaypointID', -1)

    def isNull(self):
        if self.ORB.subscribe('Waypoint') is []:
            return True
        else:
            return False

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def _log(self, msg):
        print msg

if __name__ == "__main__":
    from uORB import uORB
    ORB = uORB()
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    print wp.subscribe('Waypoint'), wp.subscribe('WaypointID')
    info = "36.1+116.1,36.2+116.2,36.3+116.3,36.4+116.4,36.5+116.5"
    wp.Route(info)
    print wp.ORB.subscribe('Waypoint'), wp.ORB.subscribe('WaypointID')
    wp.add_number()
    print wp.remain_wp(), wp.ORB.subscribe('WaypointID')
