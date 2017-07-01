#! /usr/bin/env python
# coding:utf-8

import sys
sys.path.append('..')
from concurrent import futures
import time
import grpc
import protobuf.mc_rpc_pb2 as mc
import protobuf.mc_rpc_pb2_grpc as mc_rpc
from lib.tools import exe_actions, CancelWatcher
from lib.logger import logger
from multiprocessing import Pool

_ONE_DAY_IN_SECONDS = 60 * 60 * 24


class FC_RPC(mc_grpc.FCServicer):

    def __init__(self, vehicle, lidar):
        self.vehicle = vehicle
        self.lidar = lidar
        self.pool = Pool(5)

    def ResetLoiter(self):
        id = request.id
        result = self.vehicle.set_channels_mid()
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def ControlFRU(self, request, context):
        CancelWatcher.Cancel = True
        id = request.id
        actions = request.actions
        try:
            exe_actions(self.vehicle, actions)
        except AssertionError as e:
            logger.error(e)
            result = False
        else:
            result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def SetGear(self, request, context):
        id = request.id
        index = request.index
        # result = self.vehicle.set_gear(index)
        self.pool.apply_async(self.vehicle.set_gear, args=(index,))
        result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def TargetByMetres(self, request, context):
        CancelWatcher.Cancel = True
        id = request.id
        North = request.North
        East = request.East
        result = self.vehicle.set_target(North, East)
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def TargetByAngle(self, request, context):
        CancelWatcher.Cancel = True
        id = request.id
        distance = request.distance
        angle = request.angle
        result = self.vehicle.set_target_angle(distance, angle)
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def Guided(self, request, context):
        id = request.id
        CancelWatcher = True

        # result = self.vehicle.Guided()
        self.pool.apply_async(self.vehicle.Guided)
        result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def RTL(self, request, context):
        id = request.id
        CancelWatcher = True
        # result = self.vehicle.RTL()
        self.pool.apply_async(self.vehicle.RTL)
        result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def Auto(self, request, context):
        id = request.id
        CancelWatcher = True
        # result = self.vehicle.Auto()
        self.pool.apply_async(self.vehicle.Auto)
        result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def PlanRoute(self, request, context):
        CancelWatcher = True
        id = request.id
        points = request.points
        array_points = []
        for p in points:
            array_points.append([p.latitude, p.longitude, p.altitude])
        # print array_points
        self.vehicle.publish('Waypoint', array_points)
        self.vehicle.publish('WaypointID', 0)
        # result = self.vehicle.Auto()
        result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def AI_Guided(self, request, context):
        CancelWatcher = True
        id = request.id
        if lidar is None:
            logger.critical('Lidar object is not initialized')
            result = False
        else:
            # result = self.lidar.Guided()
            self.pool.apply_async(self.lidar.Guided)
            result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def AI_RTL(self, request, context):
        CancelWatcher = True
        id = request.id
        if lidar is None:
            logger.critical('Lidar object is not initialized')
            result = False
        else:
            # result = self.lidar.RTL()
            self.pool.apply_async(self.lidar.RTL)
            result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)

    def AI_Auto(self, request, context):
        CancelWatcher = True
        id = request.id
        if lidar is None:
            logger.critical('Lidar object is not initialized')
            result = False
        else:
            # result = self.lidar.Auto()
            self.pool.apply_async(self.lidar.Auto)
            result = True
        response = {'id': id, 'ack': result}
        return mc.response(**response)


def GCS_Start(vehicle, lidar=None):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    mc_grpc.add_FCServicer_to_server(FC_RPC(vehicle, lidar), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    logger.info('GCS_RPC_SERVER start...')
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB
    from AF_Copter.vehicle import init_vehicle

    ORB = uORB()
    Watcher()

    vehicle = init_vehicle(ORB)
    from AF_Avoid.lidar_rpc import Lidar
    lidar = Lidar(vehicle)
    GCS_Start(vehicle, lidar)
