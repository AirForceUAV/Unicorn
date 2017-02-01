#!/usr/bin/python
# -*- coding=utf-8 -*-

from library import element, get_location_metres


class Waypoint(object):

    def __init__(self, ORB):
        file_path = 'waypoint.xml'
        self._root = element(file_path)
        self.ORB = ORB

    @property
    def points(self):
        return self.subscribe('Waypoint')

    @property
    def ID(self):
        return self.subscribe('WaypointID')

    @property
    def type(self):
        return self.subscribe('WaypointType')

    def download(self, origin, index=0):
        count = len(self._root.getchildren())
        if index > count - 1:
            print 'Error:index out of range when download Waypoints'
            return
        _root = self._root[index]
        Trail = _root.get('Trail')
        result = [origin]
        points = _root.getchildren()
        number = 0
        for point in points:
            result.append(get_location_metres(
                result[number], self.child(point, 0), self.child(point, 1)))
            number += 1
        self.publish('Waypoint', result[1:])
        self.publish('WaypointID', 0)
        self.publish('WaypointType', 'Download')
        print 'Trail :', Trail
        print 'Waypoints :', self.subscribe('Waypoint')

    def Route(self, info):
        if info == "":
            print "Route is None"
            return
        result = []
        wps = info.split(',')
        for wp in wps:
            loc = wp.split('+')
            result.append([float(loc[0]), float(loc[1])])
        self.publish('Waypoint', result)
        self.publish('WaypointID', 0)
        self.publish('WaypointType', 'Route')
        print 'Waypoints :', self.subscribe('Waypoint')

    def remain_wp(self):
        return self.points[self.ID:] if self.ID is not -1 else []

    def add_number(self):
        a = len(self.points) - 1
        b = self.ID + 1
        self.publish('WaypointID', min(a, b))

    def child(self, point, index):
        return float(point[index].text)

    def clear(self):
        self.publish('Waypoint', [])
        self.publish('WaypointID', -1)

    def isNull(self):
        return True if self.subscribe('WaypointID') < 0 else False

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)


if __name__ == "__main__":
    from uORB import uORB
    ORB = uORB()
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    info = "36.1+116.1,36.2+116.2,36.3+116.3,36.4+116.4,36.5+116.5"
    wp.Route(info)
    wp.add_number()
    # wp.clear()
    print ORB.subscribe('Waypoint'), ORB.subscribe('WaypointID')
