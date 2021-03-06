# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: FlightLog.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='FlightLog.proto',
  package='buffers',
  syntax='proto3',
  serialized_pb=_b('\n\x0f\x46lightLog.proto\x12\x07\x62uffers\"A\n\x08Location\x12\x10\n\x08latitude\x18\x01 \x01(\x02\x12\x11\n\tlongitude\x18\x02 \x01(\x02\x12\x10\n\x08\x61ltitude\x18\x03 \x01(\x02\"8\n\x05Point\x12\n\n\x02ID\x18\x01 \x01(\x05\x12#\n\x08location\x18\x02 \x01(\x0b\x32\x11.buffers.Location\"4\n\x08\x41ttitude\x12\r\n\x05pitch\x18\x01 \x01(\x02\x12\x0c\n\x04roll\x18\x02 \x01(\x02\x12\x0b\n\x03yaw\x18\x03 \x01(\x02\"-\n\nCoordinate\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\"L\n\x03GPS\x12\r\n\x05state\x18\x01 \x01(\x08\x12\x11\n\tnum_stars\x18\x02 \x01(\x05\x12#\n\x08location\x18\x03 \x01(\x0b\x32\x11.buffers.Location\"=\n\x07\x43ompass\x12\r\n\x05state\x18\x01 \x01(\x08\x12#\n\x08\x61ttitude\x18\x02 \x01(\x0b\x32\x11.buffers.Attitude\"S\n\tBarometre\x12\r\n\x05state\x18\x01 \x01(\x08\x12\x10\n\x08Pressure\x18\x02 \x01(\x02\x12\x13\n\x0bTemperature\x18\x03 \x01(\x02\x12\x10\n\x08\x41ltitude\x18\x04 \x01(\x02\"F\n\x08Waypoint\x12\r\n\x05index\x18\x01 \x01(\x11\x12\x1d\n\x05point\x18\x02 \x03(\x0b\x32\x0e.buffers.Point\x12\x0c\n\x04type\x18\x03 \x01(\t\"r\n\x08\x43hannels\x12\x0b\n\x03\x63h1\x18\x01 \x01(\x05\x12\x0b\n\x03\x63h2\x18\x02 \x01(\x05\x12\x0b\n\x03\x63h3\x18\x03 \x01(\x05\x12\x0b\n\x03\x63h4\x18\x04 \x01(\x05\x12\x0b\n\x03\x63h5\x18\x05 \x01(\x05\x12\x0b\n\x03\x63h6\x18\x06 \x01(\x05\x12\x0b\n\x03\x63h7\x18\x07 \x01(\x05\x12\x0b\n\x03\x63h8\x18\x08 \x01(\x05\"\xd4\x03\n\x07sensors\x12\x11\n\ttimestamp\x18\x01 \x01(\x02\x12\x19\n\x03gps\x18\x02 \x01(\x0b\x32\x0c.buffers.GPS\x12!\n\x07\x63ompass\x18\x03 \x01(\x0b\x32\x10.buffers.Compass\x12 \n\x04\x62\x61ro\x18\x04 \x01(\x0b\x32\x12.buffers.Barometre\x12#\n\x08waypoint\x18\x05 \x01(\x0b\x32\x11.buffers.Waypoint\x12!\n\x06target\x18\x06 \x01(\x0b\x32\x11.buffers.Location\x12\x1f\n\x04home\x18\x07 \x01(\x0b\x32\x11.buffers.Location\x12\x10\n\x08init_alt\x18\x08 \x01(\x02\x12\x18\n\x10\x44istanceToTarget\x18\t \x01(\x02\x12\x18\n\x10\x44istanceFromHome\x18\n \x01(\x02\x12(\n\rChannelsInput\x18\x0b \x01(\x0b\x32\x11.buffers.Channels\x12)\n\x0e\x43hannelsOutput\x18\x0c \x01(\x0b\x32\x11.buffers.Channels\x12$\n\tLoiterPWM\x18\r \x01(\x0b\x32\x11.buffers.Channels\x12\x10\n\x08\x41ltitude\x18\x0e \x01(\x02\x12\x0c\n\x04Mode\x18\x0f \x01(\t\x12\x0c\n\x04Gear\x18\x10 \x01(\x05\x62\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_LOCATION = _descriptor.Descriptor(
  name='Location',
  full_name='buffers.Location',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='latitude', full_name='buffers.Location.latitude', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='buffers.Location.longitude', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='buffers.Location.altitude', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=28,
  serialized_end=93,
)


_POINT = _descriptor.Descriptor(
  name='Point',
  full_name='buffers.Point',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ID', full_name='buffers.Point.ID', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='location', full_name='buffers.Point.location', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=95,
  serialized_end=151,
)


_ATTITUDE = _descriptor.Descriptor(
  name='Attitude',
  full_name='buffers.Attitude',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pitch', full_name='buffers.Attitude.pitch', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='roll', full_name='buffers.Attitude.roll', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='buffers.Attitude.yaw', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=153,
  serialized_end=205,
)


_COORDINATE = _descriptor.Descriptor(
  name='Coordinate',
  full_name='buffers.Coordinate',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='buffers.Coordinate.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='buffers.Coordinate.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='z', full_name='buffers.Coordinate.z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=207,
  serialized_end=252,
)


_GPS = _descriptor.Descriptor(
  name='GPS',
  full_name='buffers.GPS',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='buffers.GPS.state', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_stars', full_name='buffers.GPS.num_stars', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='location', full_name='buffers.GPS.location', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=254,
  serialized_end=330,
)


_COMPASS = _descriptor.Descriptor(
  name='Compass',
  full_name='buffers.Compass',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='buffers.Compass.state', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='attitude', full_name='buffers.Compass.attitude', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=332,
  serialized_end=393,
)


_BAROMETRE = _descriptor.Descriptor(
  name='Barometre',
  full_name='buffers.Barometre',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='buffers.Barometre.state', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Pressure', full_name='buffers.Barometre.Pressure', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Temperature', full_name='buffers.Barometre.Temperature', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Altitude', full_name='buffers.Barometre.Altitude', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=395,
  serialized_end=478,
)


_WAYPOINT = _descriptor.Descriptor(
  name='Waypoint',
  full_name='buffers.Waypoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='index', full_name='buffers.Waypoint.index', index=0,
      number=1, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='point', full_name='buffers.Waypoint.point', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='buffers.Waypoint.type', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=480,
  serialized_end=550,
)


_CHANNELS = _descriptor.Descriptor(
  name='Channels',
  full_name='buffers.Channels',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ch1', full_name='buffers.Channels.ch1', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch2', full_name='buffers.Channels.ch2', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch3', full_name='buffers.Channels.ch3', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch4', full_name='buffers.Channels.ch4', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch5', full_name='buffers.Channels.ch5', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch6', full_name='buffers.Channels.ch6', index=5,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch7', full_name='buffers.Channels.ch7', index=6,
      number=7, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ch8', full_name='buffers.Channels.ch8', index=7,
      number=8, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=552,
  serialized_end=666,
)


_SENSORS = _descriptor.Descriptor(
  name='sensors',
  full_name='buffers.sensors',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='buffers.sensors.timestamp', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps', full_name='buffers.sensors.gps', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='compass', full_name='buffers.sensors.compass', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='baro', full_name='buffers.sensors.baro', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waypoint', full_name='buffers.sensors.waypoint', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='target', full_name='buffers.sensors.target', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='home', full_name='buffers.sensors.home', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='init_alt', full_name='buffers.sensors.init_alt', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='DistanceToTarget', full_name='buffers.sensors.DistanceToTarget', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='DistanceFromHome', full_name='buffers.sensors.DistanceFromHome', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ChannelsInput', full_name='buffers.sensors.ChannelsInput', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ChannelsOutput', full_name='buffers.sensors.ChannelsOutput', index=11,
      number=12, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='LoiterPWM', full_name='buffers.sensors.LoiterPWM', index=12,
      number=13, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Altitude', full_name='buffers.sensors.Altitude', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Mode', full_name='buffers.sensors.Mode', index=14,
      number=15, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Gear', full_name='buffers.sensors.Gear', index=15,
      number=16, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=669,
  serialized_end=1137,
)

_POINT.fields_by_name['location'].message_type = _LOCATION
_GPS.fields_by_name['location'].message_type = _LOCATION
_COMPASS.fields_by_name['attitude'].message_type = _ATTITUDE
_WAYPOINT.fields_by_name['point'].message_type = _POINT
_SENSORS.fields_by_name['gps'].message_type = _GPS
_SENSORS.fields_by_name['compass'].message_type = _COMPASS
_SENSORS.fields_by_name['baro'].message_type = _BAROMETRE
_SENSORS.fields_by_name['waypoint'].message_type = _WAYPOINT
_SENSORS.fields_by_name['target'].message_type = _LOCATION
_SENSORS.fields_by_name['home'].message_type = _LOCATION
_SENSORS.fields_by_name['ChannelsInput'].message_type = _CHANNELS
_SENSORS.fields_by_name['ChannelsOutput'].message_type = _CHANNELS
_SENSORS.fields_by_name['LoiterPWM'].message_type = _CHANNELS
DESCRIPTOR.message_types_by_name['Location'] = _LOCATION
DESCRIPTOR.message_types_by_name['Point'] = _POINT
DESCRIPTOR.message_types_by_name['Attitude'] = _ATTITUDE
DESCRIPTOR.message_types_by_name['Coordinate'] = _COORDINATE
DESCRIPTOR.message_types_by_name['GPS'] = _GPS
DESCRIPTOR.message_types_by_name['Compass'] = _COMPASS
DESCRIPTOR.message_types_by_name['Barometre'] = _BAROMETRE
DESCRIPTOR.message_types_by_name['Waypoint'] = _WAYPOINT
DESCRIPTOR.message_types_by_name['Channels'] = _CHANNELS
DESCRIPTOR.message_types_by_name['sensors'] = _SENSORS

Location = _reflection.GeneratedProtocolMessageType('Location', (_message.Message,), dict(
  DESCRIPTOR = _LOCATION,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Location)
  ))
_sym_db.RegisterMessage(Location)

Point = _reflection.GeneratedProtocolMessageType('Point', (_message.Message,), dict(
  DESCRIPTOR = _POINT,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Point)
  ))
_sym_db.RegisterMessage(Point)

Attitude = _reflection.GeneratedProtocolMessageType('Attitude', (_message.Message,), dict(
  DESCRIPTOR = _ATTITUDE,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Attitude)
  ))
_sym_db.RegisterMessage(Attitude)

Coordinate = _reflection.GeneratedProtocolMessageType('Coordinate', (_message.Message,), dict(
  DESCRIPTOR = _COORDINATE,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Coordinate)
  ))
_sym_db.RegisterMessage(Coordinate)

GPS = _reflection.GeneratedProtocolMessageType('GPS', (_message.Message,), dict(
  DESCRIPTOR = _GPS,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.GPS)
  ))
_sym_db.RegisterMessage(GPS)

Compass = _reflection.GeneratedProtocolMessageType('Compass', (_message.Message,), dict(
  DESCRIPTOR = _COMPASS,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Compass)
  ))
_sym_db.RegisterMessage(Compass)

Barometre = _reflection.GeneratedProtocolMessageType('Barometre', (_message.Message,), dict(
  DESCRIPTOR = _BAROMETRE,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Barometre)
  ))
_sym_db.RegisterMessage(Barometre)

Waypoint = _reflection.GeneratedProtocolMessageType('Waypoint', (_message.Message,), dict(
  DESCRIPTOR = _WAYPOINT,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Waypoint)
  ))
_sym_db.RegisterMessage(Waypoint)

Channels = _reflection.GeneratedProtocolMessageType('Channels', (_message.Message,), dict(
  DESCRIPTOR = _CHANNELS,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.Channels)
  ))
_sym_db.RegisterMessage(Channels)

sensors = _reflection.GeneratedProtocolMessageType('sensors', (_message.Message,), dict(
  DESCRIPTOR = _SENSORS,
  __module__ = 'FlightLog_pb2'
  # @@protoc_insertion_point(class_scope:buffers.sensors)
  ))
_sym_db.RegisterMessage(sensors)


# @@protoc_insertion_point(module_scope)
