from grpc_tools import protoc

protoc.main(
    (
        '',
        '-I.',
        '--python_out=.',
        'FlightLog.proto',
    )
)