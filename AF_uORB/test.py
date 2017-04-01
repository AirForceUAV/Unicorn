def _debug(LoiterPWM):
    test = {
        'Compass_State': True,
        'Sbus_State': True,
        'Attitude': [-0.32, 0.01, 66],
        'Baro_State': True,
        'Pressure': 1013.25,
        'Temperature': 26,
        'ChannelsInput': LoiterPWM,
        'GPS_State': True,
        'Location': [36.11127966305683, 116.2222, 100],
        'NumStars': 16,
        'ChannelsOutput': LoiterPWM,
        'HomeLocation': [36.222, 116.2222, 0],
        'Gear': 1,
        'Target': None,
        'LoiterPWM': LoiterPWM,
        'Mode': 'STAB',
        'Waypoint': [
            [36.121111111111, 116.22211080524757, -1],
            [36.111200831528414, 116.2223331950068, -1],
            [36.111111, 116.22222200012719, -1],
        ],
        'WaypointID': 0,
        'RPM': 1600,
        'InitAltitude': -80.81,
        'IMU_State': True,
        'ACC': [0.1, 0.2, 0.3],
        'GYR': [0.1, 0.2, 0.3],
        'MAG': [0.1, 0.2, 0.3],
        'EUL': [0.1, 0.2, 0.3],
        'QUA': [0.1, 0.2, 0.3, 0.4]}
    return test

    # print _debug(None)
