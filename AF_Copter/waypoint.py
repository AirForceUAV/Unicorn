#!/usr/bin/python
# -*- coding=utf-8 -*-

import sys
sys.path.append('..')
from lib.science import get_location_metres
from lib.logger import logger


class Waypoint(object):

    def __init__(self, ORB):
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

    def download(self, origin, index):
        import toml
        import os
        file_path = os.path.join('..', 'Toml', 'waypoint.yaml')
        key = "waypoint{}".format(index)
        with open(file_path) as f:
            wps = toml.loads(f.read())
        count = len(wps)
        if index > count or index == 0:
            logger.error('index out of range when download Waypoints')
            return
        waypoints = wps[key]
        Trail = waypoints['Trail']
        result = [origin]
        points = waypoints['points']
        number = 0
        for point in points:
            result.append(get_location_metres(
                result[number], point[0], point[1]))
            number += 1
        self.publish('Waypoint', result[1:])
        self.publish('WaypointID', 0)
        # self.publish('WaypointType', 'Download')

        print('Trail:{}\nWaypoints:{}'.format(
            Trail, self.subscribe('Waypoint')))
        logger.info('Download complete')

    def Route(self, info):
        if info == "":
            return
        result = []
        wps = info.split(',')
        for wp in wps:
            loc = wp.split('+')
            result.append([float(loc[0]), float(loc[1])])
        self.publish('Waypoint', result)
        self.publish('WaypointID', 0)
        # self.publish('WaypointType', 'Route')

        # print 'Waypoints :', self.points
        logger.info('Route planning complete')

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
        # self.publish('WaypointType', None)

    def isNull(self):
        return True if self.subscribe('WaypointID') < 0 else False

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)


if __name__ == "__main__":
    from AF_uORB.uORB import uORB
    ORB = uORB()
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 1)
    info = "36.1+116.1,36.2+116.2,36.3+116.3,36.4+116.4,36.5+116.5"
    wp.Route(info)
    wp.add_number()
    # wp.clear()
    print ORB.subscribe('Waypoint'), ORB.subscribe('WaypointID')
