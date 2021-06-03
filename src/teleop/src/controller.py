import inputs

event_map = {
        'ABS_X':      'LEFT_JOY_X',
        'ABS_Y':      'LEFT_JOY_Y',
        'BTN_SELECT': 'BTN_LEFT_JOY',

        'ABS_Z':      'RIGHT_JOY_X',
        'ABS_RZ':     'RIGHT_JOY_Y',
        'BTN_START':  'BTN_RIGHT_JOY',

        'ABS_HAT0X':  'DPAD_X',
        'ABS_HAT0Y':  'DPAD_Y',

        'BTN_TL':     'BTN_ZL',
        'BTN_TR':     'BTN_ZR',
        'BTN_WEST':   'BTN_L',
        'BTN_Z':      'BTN_R',

        'BTN_TL2':    'BTN_MINUS',
        'BTN_TR2':    'BTN_PLUS',
        'BTN_THUMBL': 'BTN_CIRCLE',
        'BTN_MODE':   'BTN_HOME',

        'BTN_C': 'BTN_A',
        'BTN_EAST': 'BTN_B',
        'BTN_SOUTH': 'BTN_Y',
        'BTN_NORTH': 'BTN_X'
}

class Controller:
    def __init__(self):
        self.state = {}
        self.state['LEFT_JOY_X'] = 128
        self.state['LEFT_JOY_Y'] = 128
        self.state['RIGHT_JOY_X'] = 128
        self.state['RIGHT_JOY_Y'] = 128

        try:
            self.gamepad = inputs.devices.gamepads[0]
        except IndexError:
            raise inputs.UnpluggedError("No gamepad found.")

    def update(self):
        events = self.gamepad.read()
        for event in events:
            # Skip miscellaneous events.
            if event.ev_type != 'Key' and event.ev_type != 'Absolute':
                continue

            # Remap event codes.
            if event.code in event_map:
                event.code = event_map[event.code]
            else:
                print("Error: Key code {} not mapped.\n".format(event.code))
                continue

            # Update the controller state.
            self._handle_event(event.code, event.state)

    def lookup(self, code):
        if code not in self.state:
            self.state[code] = 0.0
        return self.state[code]

    def _handle_event(self, code, state):
        self.state[code] = state

