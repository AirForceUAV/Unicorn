#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import for_mc_pb2 as mc
from GCS_Proto import deserialize, parse_proto
from lib.logger import logger


def Test_Common():
    _object = mc.SendCommand()
    _object.timestamp = time.time()
    return _object


def Test_ResetLoiter():
    specified = mc.ResetLoiter()
    _object = Test_Common()
    _object.code = mc._ResetLoiter
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_SetGear():
    specified = mc.SetGear()
    specified.index = 2
    _object = Test_Common()
    _object.code = mc._SetGear
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_Cancel():
    specified = mc.Cancel()
    _object = Test_Common()
    _object.code = mc._Cancel
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_TargetByMetres():
    specified = mc.TargetByMetres()
    specified.North = 100
    specified.East = -20
    _object = Test_Common()
    _object.code = mc._TargetByMetres
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_TargetByAngle():
    specified = mc.TargetByAngle()
    specified.distance = 100
    specified.angle = 20
    _object = Test_Common()
    _object.code = mc._TargetByAngle
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_PlanRoute():
    from collections import namedtuple
    Location = namedtuple('Location', ['latitude', 'longitude', 'altitude'])
    _points = (Location(34, 119, 10),
               Location(35, 119, 10),
               Location(36, 119, 10))
    specified = mc.PlanRoute()
    for p in _points:
        point = specified.points.add()
        point.latitude = p.latitude
        point.longitude = p.longitude
        point.altitude = p.altitude

    _object = Test_Common()
    _object.code = mc._PlanRoute
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_ControlFRU():
    specified = mc.ControlFRU()
    _actions = [mc.FORWARD, mc.LEFT_ROLL]
    for a in _actions:
        action = specified.actions.add()
        action.action = a

    _object = Test_Common()
    _object.code = mc._ControlFRU
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_Guided():
    specified = mc.Guided()
    _object = Test_Common()
    _object.code = mc._Guided
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_RTL():
    specified = mc.RTL()
    _object = Test_Common()
    _object.code = mc._Rtl
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_Auto():
    specified = mc.Auto()
    _object = Test_Common()
    _object.code = mc._Auto
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_AI_Guided():
    specified = mc.AI_Guided()
    _object = Test_Common()
    _object.code = mc._AI_Guided
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_AI_RTL():
    specified = mc.AI_RTL()
    _object = Test_Common()
    _object.code = mc._AI_Rtl
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_AI_Auto():
    specified = mc.AI_Auto()
    _object = Test_Common()
    _object.code = mc._AI_Auto
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string


def Test_Semi_Auto():
    specified = mc.Semi_Auto()
    _actions = [mc.FORWARD, mc.LEFT_ROLL]
    for a in _actions:
        action = specified.actions.add()
        action.action = a
    _object = Test_Common()
    _object.code = mc._Semi_Auto
    _object.command.Pack(specified)
    proto_string = _object.SerializeToString()
    return proto_string

if __name__ == "__main__":
    from AF_Copter.vehicle import Vehicle
    from AF_uORB.uORB import uORB

    ORB = uORB()

    vehicle = Vehicle(ORB)
    lidar = None

    _parser = parse_proto(vehicle, lidar)

    messages = [
        # Test_ResetLoiter(),
        # Test_SetGear(),
        # Test_Cancel(),
        # Test_TargetByMetres(),
        # Test_TargetByAngle(),
        # Test_PlanRoute(),
        # Test_ControlFRU(),
        # Test_Guided(),
        # Test_RTL(),
        # Test_Auto(),
        # Test_AI_Guided(),
        # Test_AI_RTL(),
        # Test_AI_Auto(),
        Test_Semi_Auto(),
    ]

    for message in messages:
        cmd_object = deserialize(message)
        try:
            _parser(cmd_object)
        except Exception as e:
            logger.error(e)
