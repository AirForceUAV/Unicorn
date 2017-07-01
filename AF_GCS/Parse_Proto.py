#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('..')
import os
import time
from lib.logger import logger
import protobuf.for_mc_pb2 as mc


def Parse_Actions(actions):
    enum2action = {
        mc.STOP: {},
        mc.FORWARD: {'ELE': 1}, mc.BACKWARD: {'ELE': -1},
        mc.RIGHT_YAW: {'RUD': 1}, mc.LEFT_YAW: {'RUD': -1},
        mc.RIGHT_ROLL: {'AIL': 1}, mc.LEFT_ROLL: {'AIL': -1},
        mc.UP: {'THR': 1}, mc.DOWN: {'THR': -1},
    }
    result = {}
    for a in actions:
        action = a.action

        if action == mc.STOP:
            result = {}
            break
        dictaction = enum2action.get(action)
        assert not (dictaction is None or dictaction.keys()[0] in result),\
            'Command is invalid. Note: Command is {}'.format(actions)
        result = dict(result, **dictaction)
    return result


def deserialize(message):
    cmd_object = mc.SendCommand()
    cmd_object.ParseFromString(message)
    return cmd_object


def parse_proto(vehicle, lidar):

    def Parser(cmd_object):
        code = cmd_object.code
        any = cmd_object.command
        _parser = code2parser.get(code)
        assert _parser != None,\
            'code:{} is invalid'.format(code)
        # print type(_parser)
        _parser(any)

    def Parse_ResetLoiter(any):
        specified = mc.ResetLoiter()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message ResetLoiter'
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.set_channels_mid()')
        vehicle.set_channels_mid()

    def Parse_SetGear(any):
        specified = mc.SetGear()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message SetGear'
        any.Unpack(specified)
        index = specified.index
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.set_gear({})'.format(index))
        vehicle.set_gear(index)

    def Parse_Cancel(any):
        specified = mc.Cancel()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message Cancel'
        logger.debug('Execute Command:Cancel')
        CancelWatcher.Cancel = True

    def Parse_TargetByMetres(any):
        specified = mc.TargetByMetres()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message TargetByMetres'
        any.Unpack(specified)
        North = specified.North
        East = specified.East
        assert vehicle != None, 'vehicle is null'
        logger.debug(
            'Execute Command: vehicle.set_target({},{})'.format(North, East))
        vehicle.set_target(North, East)

    def Parse_TargetByAngle(any):
        specified = mc.TargetByAngle()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message TargetByAngle'
        any.Unpack(specified)
        distance = specified.distance
        angle = specified.angle
        assert vehicle != None, 'vehicle is null'
        logger.debug(
            'Execute Command: vehicle.set_target_angle({},{})'.format(distance, angle))
        vehicle.set_target_angle(distance, angle)

    def Parse_PlanRoute(any):
        specified = mc.PlanRoute()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message PlanRoute'
        any.Unpack(specified)
        points = specified.points
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle._Route()')
        # for point in points:
        #     print point.latitude, point.longitude, point.altitude
        vehicle._Route(points)

    def Parse_ControlFRU(any):
        specified = mc.ControlFRU()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message ControlFRU'
        any.Unpack(specified)
        actions = specified.actions
        kwargs = Parse_Actions(actions)
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.Control_FRU({})'.format(kwargs))
        vehicle.control_FRU(**kwargs)

    def Parse_Guided(any):
        specified = mc.Guided()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message Guided'
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.Guided()')
        vehicle.Guided()

    def Parse_RTL(any):
        specified = mc.RTL()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message RTL'
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.RTL()')
        vehicle.RTL()

    def Parse_Auto(any):
        specified = mc.Auto()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message Auto'
        assert vehicle != None, 'vehicle is null'
        logger.debug('Execute Command: vehicle.Auto()')
        vehicle.Auto()

    def Parse_AI_Guided(any):
        specified = mc.AI_Guided()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message AI_Guided'
        assert lidar != None, 'liar is null'
        logger.debug('Execute Command: lidar.Guided()')
        lidar.Guided()

    def Parse_AI_RTL(any):
        specified = mc.AI_RTL()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message AI_RTL'
        assert lidar != None, 'liar is null'
        logger.debug('Execute Command: lidar.RTL()')
        lidar.RTL()

    def Parse_AI_Auto(any):
        specified = mc.AI_Auto()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message AI_Auto'
        assert lidar != None, 'lidar is null'
        logger.debug('Execute Command: lidar.Auto()')
        lidar.Auto()

    def Parse_Semi_Auto(any):
        specified = mc.Semi_Auto()
        assert any.Is(specified.DESCRIPTOR), \
            'Any is not message Semi_Auto'

        any.Unpack(specified)
        actions = specified.actions
        kwargs = Parse_Actions(actions)
        assert lidar != None, 'lidar is null'
        logger.debug('Execute Command: lidar.Semi_Auto({})'.format(kwargs))

        lidar.Semi_Auto(**kwargs)

    code2parser = {mc._ResetLoiter: Parse_ResetLoiter,
                   mc._SetGear: Parse_SetGear,
                   mc._Cancel: Parse_Cancel,
                   mc._TargetByMetres: Parse_TargetByMetres,
                   mc._TargetByAngle: Parse_TargetByAngle,
                   mc._PlanRoute: Parse_PlanRoute,
                   mc._ControlFRU: Parse_ControlFRU,
                   mc._Guided: Parse_Guided,
                   mc._Rtl: Parse_RTL,
                   mc._Auto: Parse_Auto,
                   mc._AI_Guided: Parse_AI_Guided,
                   mc._AI_Rtl: Parse_AI_RTL,
                   mc._AI_Auto: Parse_AI_Auto,
                   mc._Semi_Auto: Parse_Semi_Auto, }
    return Parser
