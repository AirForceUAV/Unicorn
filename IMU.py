#!/usr/bin/evn python
# coding:utf-8

from sense_hat import SenseHat

sense = SenseHat()
sense.set_imu_config(True, True, True)

pressure = sense.get_pressure()
print("Pressure: %s Millibars" % pressure)

# alternatives
print(sense.pressure)

# orientation_rad = sense.get_orientation_radians()
# print "orientation_radians"
# print(" p: {pitch}, r: {roll}, y: {yaw}".format(
#     **orientation_rad))
# # alternatives
# print(sense.orientation_radians)

# orientation = sense.get_orientation_degrees()
# print "orientation_degrees"
# print(" p: {pitch}, r: {roll}, y: {yaw}".format(
#     **orientation))

# orientation = sense.get_orientation()
# print"orientation "
# print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))
# # alternatives
# print(sense.orientation)

north = sense.get_compass()
print("North: %s" % north)
# alternatives
print(sense.compass)

# raw = sense.get_compass_raw()
# print "compass_raw "
# print("x: {x}, y: {y}, z: {z}".format(**raw))

# gyro_only = sense.get_gyroscope()
# print "gyroscope"
# print(" p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))
# # alternatives
# print(sense.gyro)
# print(sense.gyroscope)
# # alternatives
print(sense.compass_raw)

# raw = sense.get_gyroscope_raw()
# print "gyroscope_raw"
# print("x: {x}, y: {y}, z: {z}".format(**raw))
# # alternatives
# print(sense.gyro_raw)
# print(sense.gyroscope_raw)

# accel_only = sense.get_accelerometer()
# print "accelerometer"
# print("p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))
# # alternatives
# print(sense.accel)
# print(sense.accelerometer)

# raw = sense.get_accelerometer_raw()
# print "accelerometer_raw"
# print(" x: {x}, y: {y}, z: {z}".format(**raw))
# # alternatives
# print(sense.accel_raw)
# print(sense.accelerometer_raw)
