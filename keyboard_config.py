keyboard_event = {'up': 'FORWARD', 'down': 'BACKWARD',
                  'left': 'LEFT_ROLL', 'right': 'RIGHT_ROLL',
                  'page up': 'UP', 'page down': 'DOWN',
                  'space': 'STOP', 'esc': 'esc'}

map_event_args = {'FORWARD': ('ELE', 1), 'BACKWARD': ('ELE', -1),
                  'LEFT_ROLL': ('AIL', -1), 'RIGHT_ROLL': ('AIL', 1),
                  'UP': ('THR', 1), 'DOWN': ('THR', -1),
                  'LEFT_YAW': ('RUD', -1), 'RIGHT_YAW': ('RUD', 1),
                  'STOP': None
                  }
