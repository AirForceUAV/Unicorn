from grpc_tools import protoc

protoc.main(
    (
        '',
        '-I.',
        '--python_out=.',
        'FlightLog.proto',
    )
)

# protoc.main(
#     (
#         '',
#         '-I.',
#         '--python_out=.',
#         # '--plugin=protoc-gen-grpc="which grpc_python_plugin"',
#         'for_mc.proto',
#     )
# )

protoc.main(
    (
        '',
        '-I.',
        '--python_out=.',
        '--grpc_python_out=.',
        'mc_rpc.proto',
    )
)

protoc.main(
    (
        '',
        '-I.',
        '--python_out=.',
        '--grpc_python_out=.',
        'oa_rpc.proto',
    )
)


# protoc -I. --python_out=. FlightLog.proto
# protoc -I. --python_out=. for_mc.proto
# python -m grpc_tools.protoc -I. --python_out= . --grpc_python_out=.
# mc_rpc.proto

# python -m grpc_tools.protoc -I. --python_out= . --grpc_python_out=.
# oa_rpc.proto
