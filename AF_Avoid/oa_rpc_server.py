#! /usr/bin/env python
# coding:utf-8

import sys
sys.path.append('..')
from concurrent import futures
import time
import grpc
import protobuf.oa_rpc_pb2 as oa
import protobuf.oa_rpc_pb2_grpc as oa_grpc
from lib.config import config
_ONE_DAY_IN_SECONDS = 60 * 60 * 24


class Vehicle(oa_grpc.ObstacleAvoidanceServicer):

    def FullAutomatic(self, request, context):
        id = request.id
        actions = request.current
        print id, actions
        response = {'id': id,
                    'actions': [1]}
        # print response
        return oa.Strategy(**response)

    def SemiAutomatic(self, request, context):
        id = request.id
        actions = request.actions
        print id, actions
        response = {'id': id,
                    'actions': actions}
        # print response
        return oa.Strategy(**response)


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    oa_grpc.add_ObstacleAvoidanceServicer_to_server(Vehicle(), server)
    server.add_insecure_port('[::]:' + config.OA_rpc_port)
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    serve()
