# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc
from grpc.framework.common import cardinality
from grpc.framework.interfaces.face import utilities as face_utilities

import oa_rpc_pb2 as oa__rpc__pb2


class ObstacleAvoidanceStub(object):

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.FullAutomatic = channel.unary_unary(
        '/oa_rpc.ObstacleAvoidance/FullAutomatic',
        request_serializer=oa__rpc__pb2.FullContext.SerializeToString,
        response_deserializer=oa__rpc__pb2.Strategy.FromString,
        )
    self.SemiAutomatic = channel.unary_unary(
        '/oa_rpc.ObstacleAvoidance/SemiAutomatic',
        request_serializer=oa__rpc__pb2.SemiContext.SerializeToString,
        response_deserializer=oa__rpc__pb2.Strategy.FromString,
        )


class ObstacleAvoidanceServicer(object):

  def FullAutomatic(self, request, context):
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def SemiAutomatic(self, request, context):
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_ObstacleAvoidanceServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'FullAutomatic': grpc.unary_unary_rpc_method_handler(
          servicer.FullAutomatic,
          request_deserializer=oa__rpc__pb2.FullContext.FromString,
          response_serializer=oa__rpc__pb2.Strategy.SerializeToString,
      ),
      'SemiAutomatic': grpc.unary_unary_rpc_method_handler(
          servicer.SemiAutomatic,
          request_deserializer=oa__rpc__pb2.SemiContext.FromString,
          response_serializer=oa__rpc__pb2.Strategy.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'oa_rpc.ObstacleAvoidance', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
