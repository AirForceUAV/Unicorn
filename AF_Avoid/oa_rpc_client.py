#! /usr/bin/env python
# coding:utf-8

import sys
sys.path.append("..")
import grpc
import protobuf.oa_rpc_pb2 as oa
import protobuf.oa_rpc_pb2_grpc as oa_grpc
from lib.config import config


class OA_Stub:

    def __init__(self):
        channel = grpc.insecure_channel(
            config.OA_rpc_host + ':' + config.OA_rpc_port)
        self.stub = oa_grpc.ObstacleAvoidanceStub(channel)
        self.timeout = 5

    def FullAuto(self, context):
        response = self.stub.FullAutomatic.future(
            oa.FullContext(**context), timeout=self.timeout).result()
        return response.id, response.actions

    def SemiAuto(self, control):
        response = self.stub.SemiAutomatic.future(
            oa.SemiContext(**control), timeout=self.timeout).result()
        return response.id, response.actions

if __name__ == '__main__':
    stub = OA_Stub()
    stub2 = OA_Stub()
    context = {'id':  1,
               'target': 10,
               'epsilon': 20,
               'current': [oa.STOP],
               'last_state': [oa.STOP],
               'last_previous_state': [oa.STOP]}
    control = {'id':  2,
               'actions': [oa.FORWARD, oa.LEFT_ROLL]}
    try:
        id, actions = stub.FullAuto(context)
        print id, actions

        id, actions = stub2.SemiAuto(control)
        print id, actions
    except grpc.RpcError, e:
        print e
