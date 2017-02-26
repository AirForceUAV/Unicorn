#!/usr/bin/evn python
# coding:utf-8

import toml


def main_channel(ch):
    num = ch[0] - 1
    low = ch[1]
    mid = ch[2]
    hig = ch[3]
    var = hig - low
    sign = ch[4]
    rate = ch[5]
    return [num, low, mid, hig, var, sign, rate]


def aux_channel(ch):
    return [ch[0] - 1] + ch[1:]


def init_drone(conf):
    drone = dict()
    chs = dict()

    system = "System{}".format(conf["SystemID"])
    UAV_config = conf[system]
    # print UAV_config
    drone['UAV'] = UAV_config['UAV']
    drone['Model'] = UAV_config['Model']
    drone['MainController'] = UAV_config['MainController']
    drone['Gear'] = UAV_config["Gear"]

    chs['AIL'] = main_channel(UAV_config['AIL'])
    chs['ELE'] = main_channel(UAV_config['ELE'])
    chs['THR'] = main_channel(UAV_config['THR'])
    chs['RUD'] = main_channel(UAV_config['RUD'])
    chs['Mode'] = aux_channel(UAV_config['Mode'])
    chs['Switch'] = aux_channel(UAV_config['Switch'])

    if drone['Model'] == 'HELI':
        chs['Rate'] = aux_channel(UAV_config['Rate'])
        chs['PIT'] = main_channel(UAV_config['PIT'])
    else:
        chs['Aux1'] = aux_channel(UAV_config['Aux1'])
        chs['Aux2'] = aux_channel(UAV_config['Aux2'])
    return drone, chs


def has_module(module):
    return module in open_module


def get_sbus(config):
    import serial
    _sbus = {}
    _sbus['port'] = config['sbus']['port']
    _sbus['baudrate'] = config['sbus']['baudrate']
    _sbus['parity'] = serial.PARITY_EVEN
    _sbus['stopbits'] = serial.STOPBITS_TWO
    _sbus['bytesize'] = serial.EIGHTBITS
    return _sbus


def get_uart(sensor, conifg):
    return (config[sensor][port], config[sensor][baudrate])


def sock(config):
    return (config['MQTT']['host'], config['MQTT']['port'])


with open("config.yaml") as f:
    conf = toml.loads(f.read())
# print conf

version = conf['version']
drone, channels = init_drone(conf)

open_module = conf['open_module']
commands = conf['commands']

mqtt_socket = sock(conf)
client_id = conf['client_id']

context_topic = str(conf['topic']['publish']['full'])
control_topic = str(conf['topic']['publish']['semi'])
full_auto_topic = str(conf['topic']['subscribe']['full'])
semi_auto_topic = str(conf['topic']['subscribe']['semi'])

sbus_serial = get_sbus(conf)
compass_serial = conf['compass']['port']
GPS_serial = conf['GPS']['port']
IMU_serial = conf['IMU']['port']

if __name__ == '__main__':
    print context_topic, control_topic, full_auto_topic, semi_auto_topic
