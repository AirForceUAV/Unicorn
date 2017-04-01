from grpc_tools import protoc

protoc.main(
    (
        '',
        '-I.',
        '--python_out=.',
        'for_mc.proto',
    )
)

# protoc.main(
#     (
#         '',
#         '-I.',
#         '--python_out=.',
#         '--grpc_python_out=.',
#         'mc_rpc.proto',
#     )
# )
