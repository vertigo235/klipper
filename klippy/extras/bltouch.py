# BLTouch support
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import homing, probe

SIGNAL_PERIOD = 0.025600
MIN_CMD_TIME = 4 * SIGNAL_PERIOD

TEST_TIME = 5 * 60.
ENDSTOP_REST_TIME = .001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

Commands = {
    None: 0.0, 'pin_down': 0.000700, 'touch_mode': 0.001200,
    'pin_up': 0.001500, 'self_test': 0.001800, 'reset': 0.002200,
}

# BLTouch "endstop" wrapper
class BLTouchEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = config.getfloat('z_offset')
        # Create a pwm object to handle the control pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', config.get('control_pin'))
        self.mcu_pwm.setup_max_duration(0.)
        self.mcu_pwm.setup_cycle_time(SIGNAL_PERIOD)
        # Create an "endstop" object to handle the sensor pin
        pin = config.get('sensor_pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        mcu.register_config_callback(self._build_config)
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        # Setup for sensor test
        self.next_test_time = 0.
        self.test_sensor_pin = config.getboolean('test_sensor_pin', True)
        # Calculate pin move time
        pmt = max(config.getfloat('pin_move_time', 0.200), MIN_CMD_TIME)
        self.pin_move_time = math.ceil(pmt / SIGNAL_PERIOD) * SIGNAL_PERIOD
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        self.query_endstop_wait = self.mcu_endstop.query_endstop_wait
        self.TimeoutError = self.mcu_endstop.TimeoutError
        # Register BLTOUCH_DEBUG command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("BLTOUCH_DEBUG", self.cmd_BLTOUCH_DEBUG,
                                    desc=self.cmd_BLTOUCH_DEBUG_help)
    def _build_config(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers('Z'):
            stepper.add_to_endstop(self)
    def send_cmd(self, print_time, cmd):
        self.mcu_pwm.set_pwm(print_time, Commands[cmd] / SIGNAL_PERIOD)
    def test_sensor(self):
        if not self.test_sensor_pin:
            return
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if print_time < self.next_test_time:
            self.next_test_time = print_time + TEST_TIME
            return
        # Raise the bltouch probe and test if probe is raised
        self.send_cmd(print_time, 'reset')
        home_time = print_time + self.pin_move_time
        self.send_cmd(home_time, 'touch_mode')
        self.send_cmd(home_time + MIN_CMD_TIME, None)
        # Perform endstop check to verify bltouch reports probe raised
        prev_positions = [s.get_commanded_position()
                          for s in self.mcu_endstop.get_steppers()]
        self.mcu_endstop.home_start(home_time, ENDSTOP_SAMPLE_TIME,
                                    ENDSTOP_SAMPLE_COUNT, ENDSTOP_REST_TIME)
        try:
            self.mcu_endstop.home_wait(home_time + MIN_CMD_TIME)
        except self.mcu_endstop.TimeoutError as e:
            raise homing.EndstopError("BLTouch sensor test failed")
        for s, pos in zip(self.mcu_endstop.get_steppers(), prev_positions):
            s.set_commanded_position(pos)
        # Test was successful
        self.next_test_time = home_time + TEST_TIME
        toolhead.reset_print_time(home_time + 2. * MIN_CMD_TIME)
    def home_prepare(self):
        self.test_sensor()
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self.send_cmd(print_time, 'pin_down')
        self.send_cmd(print_time + self.pin_move_time, 'touch_mode')
        toolhead.dwell(self.pin_move_time + MIN_CMD_TIME)
        self.mcu_endstop.home_prepare()
    def home_finalize(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self.send_cmd(print_time, 'reset')
        self.send_cmd(print_time + MIN_CMD_TIME, 'pin_up')
        self.send_cmd(print_time + MIN_CMD_TIME + self.pin_move_time, None)
        toolhead.dwell(self.pin_move_time + MIN_CMD_TIME)
        self.mcu_endstop.home_finalize()
    def home_start(self, print_time, sample_time, sample_count, rest_time):
        rest_time = min(rest_time, ENDSTOP_REST_TIME)
        self.mcu_endstop.home_start(
            print_time, sample_time, sample_count, rest_time)
    def get_position_endstop(self):
        return self.position_endstop
    cmd_BLTOUCH_DEBUG_help = "Send a command to the bltouch for debugging"
    def cmd_BLTOUCH_DEBUG(self, params):
        cmd = self.gcode.get_str('COMMAND', params, None)
        if cmd is None or cmd not in Commands:
            self.gcode.respond_info("BLTouch commands: %s" % (
                ", ".join(sorted([c for c in Commands if c is not None]))))
            return
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        msg = "Sending BLTOUCH_DEBUG COMMAND=%s" % (cmd,)
        self.gcode.respond_info(msg)
        logging.info(msg)
        self.send_cmd(print_time, cmd)
        self.send_cmd(print_time + self.pin_move_time, None)
        toolhead.dwell(self.pin_move_time + MIN_CMD_TIME)

def load_config(config):
    blt = BLTouchEndstopWrapper(config)
    config.get_printer().add_object('probe', probe.PrinterProbe(config, blt))
    return blt
