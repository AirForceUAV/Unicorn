#!/usr/bin/evn python
# coding:utf-8

from library import Singleton
import json


class uORB(object):
    __metaclass__ = Singleton

    def __init__(self):
        self._HAL = {}

    def publish(self, topic, value):
        self._HAL[topic] = value

    def subscribe(self, topic):
        return self._HAL[topic]

    def has_sensor(self, sensor):
        if self._HAL[sensor][0] > 0:
            return True
        else:
            return False

    def __str__(self):
        return json.dumps(self._HAL)
