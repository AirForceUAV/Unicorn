#! /usr/bin/env python
# coding:utf-8

import grpc
import mc_rpc_pb2
import mc_rpc_pb2_grpc


class GCS_Stub:

    def __init__(self):
        channel = grpc.insecure_channel('localhost:50051')
        self.stub = mc_rpc_pb2_grpc.FCStub(channel)

    def ControlFRU(self, context):
        response = self.stub.ControlFRU.future(
            mc_rpc_pb2.movement(**context), timeout=5).result()
        return response.id, response.ack

    def SetGear(self, context):
        response = self.stub.SetGear.future(
            mc_rpc_pb2.Gear(**context), timeout=5).result()
        return response.id, response.ack

    def PlanRoute(self, context):
        response = self.stub.PlanRoute.future(
            mc_rpc_pb2.waypoints(**context), timeout=5).result()
        return response.id, response.ack

    def TargetByMetres(self, context):
        response = self.stub.TargetByMetres.future(
            mc_rpc_pb2.DistanceMetres(**context), timeout=5).result()
        return response.id, response.ack


def test_ControlFRU():
    # return {'id': 1, 'actions': [1, 2]}
    return {'id': 1, 'actions': [1, 16]}


def test_SetGear():
    # return {'id': 2, 'index': 0}
    return {'id': 2, 'index': 2}


def test_PlanRoute():
    points = [{'latitude': 32, 'longitude': 119, 'altitude': 10},
              {'latitude': 33, 'longitude': 119, 'altitude': 10},
              {'latitude': 34, 'longitude': 119, 'altitude': 10}]
    waypoints = []
    for point in points:
        p = mc_rpc_pb2.point(**point)
        waypoints.append(p)
    # print waypoints
    result = {'id': 3, 'points': waypoints}
    return result


def test_TargetByMetres():
    result = {'id': 4, 'North': -20, 'East': 0}
    return result


def test_TargetByAngle():
    result = {'id': 4, 'distance': 20, 'angle': 10}
    return result


if __name__ == '__main__':
    stub = GCS_Stub()
    try:
        # id, ACK = stub.ControlFRU(test_ControlFRU())
        # print id, ACK
        id, ACK = stub.PlanRoute(test_PlanRoute())
        print id, ACK
        # id, ACK = stub.SetGear(test_SetGear())
        # print id, ACK
        # id, ACK = stub.TargetByMetres(test_TargetByMetres())
        # print id, ACK
    except grpc.RpcError, e:
        print e
