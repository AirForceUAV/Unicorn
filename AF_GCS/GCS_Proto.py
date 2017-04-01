#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('..')
import os
import time
import socket
import threading
from lib.tools import CancelWatcher
from lib.logger import logger
import for_mc_pb2 as mc


def open_sock():
    server_address = os.path.join(os.path.expanduser('~'), '.UDS_fc')
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)
        return sock
    except socket.error as msg:
        logger.error("{}:{}".format(sys.stderr, msg))
        sys.exit(1)


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


def send_Log(sock, ORB):
    message = ORB.dataflash()
    sock.send(message)


class Receiver(threading.Thread):

    def __init__(self, work_queue, sock, parser):
        super(Receiver, self).__init__(name="Receiver")
        self.timeout = 2
        self.work_queue = work_queue
        self.sock = sock
        self.parser = parser

    def run(self):
        logger.info('Start Receiver Thread')
        concurrent_command = [mc._SetGear]
        buffer_size = 4096
        while True:
            # use this to receive command
            message = self.sock.recv(buffer_size).strip()

            if not message or message is '':
                continue
            try:
                cmd_object = deserialize(message)
                logger.debug('Receive Command:\n{}'.format(cmd_object))
            except Exception as e:
                logger.error(e)
                continue
            time_stamp = cmd_object.timestamp
            timeout = time.time() - time_stamp
            if timeout > self.timeout:
                logger.warn(
                    'Command is out-of-date,timeout:{}'.format(timeout))
                continue
            code = cmd_object.code
            if code in concurrent_command:
                self.work_queue.put(cmd_object)
            else:
                CancelWatcher.Cancel = True
                time.sleep(.1)
                try:
                    self.parser(cmd_object)
                except Exception as e:
                    logger.error(e)


class Executor(threading.Thread):

    def __init__(self, work_queue, parser):
        super(Executor, self).__init__(name="Executor")
        self.work_queue = work_queue
        self.vehicle = vehicle
        self.lidar = lidar
        self.parser = parser

    def run(self):
        logger.info('Start Executor Thread')
        while True:
            command = self.work_queue.get().strip()
            if not command or command is '':
                continue
            try:
                self.parser(cmd_object)
            except Exception as e:
                logger.error(e)


def GCS_start(ORB, vehicle=None, lidar=None):
    from apscheduler.schedulers.background import BackgroundScheduler
    import Queue
    scheduler = BackgroundScheduler()

    sock = open_sock()
    work_queue = Queue.Queue()
    parser = parse_proto(vehicle, lidar)

    receiver = Receiver(work_queue, sock, parser)
    receiver.daemon = True
    receiver.start()

    executor = Executor(work_queue, parser)
    executor.daemon = True
    executor.start()

    scheduler.add_job(send_Log, 'interval', args=(sock, ORB), seconds=1)

    scheduler.start()
    # executor.join()
    # receiver.join()


if __name__ == "__main__":
    from AF_Copter.vehicle import Vehicle
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    vehicle = Vehicle(ORB)
    lidar = None

    GCS_start(ORB, vehicle, lidar)
    while True:
        time.sleep(100)
