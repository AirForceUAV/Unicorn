#! /usr/bin/env python
# coding:utf-8

import grpc
import oa_rpc_pb2
import oa_rpc_pb2_grpc
from lib.config import config

class OA_Stub:

    def __init__(self):
        channel = grpc.insecure_channel(config.OA_rpc_host+':'+config.OA_rpc_port)
        self.stub = oa_rpc_pb2_grpc.ObstacleAvoidanceStub(channel)
        self.timeout = 5

    def FullAuto(self, context):
        response = self.stub.FullAutomatic.future(
            oa_rpc_pb2.FullContext(**context), timeout=self.timeout).result()
        return response.id, response.actions

    def SemiAuto(self, control):
        response = self.stub.SemiAutomatic.future(
            oa_rpc_pb2.SemiContext(**control), timeout=self.timeout).result()
        return response.id, response.actions

if __name__ == '__main__':
    stub = OA_Stub()
    stub2 = OA_Stub()
    context = {'id':  1,
               'target': 0,
               'epsilon': 0,
               'current': [0],
               'last_state': [0],
               'last_previous_state': [0]}
    control = {'id':  2,
               'actions': [1, 2]}
    try:
        id, actions = stub.FullAuto(context)
        print id, actions

        # id, actions = stub2.SemiAuto(control)
        # print id, actions
    except grpc.RpcError, e:
        print e
