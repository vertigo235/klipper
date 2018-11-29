# TMC2130 additional configuration options
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, pins

# Constants for sine wave correction
TMC_WAVE_FACTOR_MIN = 1.005
TMC_WAVE_FACTOR_MAX = 1.2
TMC_WAVE_AMP = 247

Registers = {
    "GCONF": 0x00, "GSTAT": 0x01, "IOIN": 0x04, "IHOLD_IRUN": 0x10,
    "TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13, "TCOOLTHRS": 0x14,
    "THIGH": 0x15, "XDIRECT": 0x2d, "MSLUT0": 0x60, "MSLUT1": 0x61,
    "MSLUT2": 0x62, "MSLUT3": 0x63, "MSLUT4": 0x64, "MSLUT5": 0x65,
    "MSLUT6": 0x66, "MSLUT7": 0x67, "MSLUTSEL": 0x68, "MSLUTSTART": 0x69,
    "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c, "COOLCONF": 0x6d,
    "DCCTRL": 0x6e, "DRV_STATUS": 0x6f, "PWMCONF": 0x70,
    "PWM_SCALE": 0x71, "ENCM_CTRL": 0x72, "LOST_STEPS": 0x73,
}

class TMC2130_EXTRA:
    def __init__(self, config, tmc2130):
        self.printer = config.get_printer()
        self._stepper = None
        self.tmc2130 = tmc2130
        self.name = tmc2130.name
        self.get_microsteps = tmc2130.get_microsteps
        self.get_phase = tmc2130.get_phase
        self.set_register = tmc2130.set_register
        self.get_register = tmc2130.get_register
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "TMC_SET_WAVE", "STEPPER", self.name,
            self.cmd_TMC_SET_WAVE, desc=self.cmd_TMC_SET_WAVE_help)
        gcode.register_mux_command(
            "TMC_SET_STEP", "STEPPER", self.name,
            self.cmd_TMC_SET_STEP)
        gcode.register_mux_command(
            "TMC_SET_CURRENT", "STEPPER", self.name,
            self.cmd_TMC_SET_CURRENT, desc=self.cmd_TMC_SET_CURRENT_help)
        wave_factor = config.getfloat('linearity_correction', 0.,
                                      minval=0., maxval=1.2)
        self._set_wave(wave_factor)
    def printer_state(self, state):
        # TODO: It might be better to look up the correct stepper
        # in the TMC_SET_STEP gcode rather than store it initially
        if state == "ready":
            toolhead = self.printer.lookup_object('toolhead')
            if self.name == 'extruder':
                self._stepper = toolhead.get_extruder().stepper
                logging.info("TMC2130 %s: Stepper Found" % (self.name))
            else:
                steppers = toolhead.get_kinematics().get_steppers()
                for s in steppers:
                    if s.get_name() == self.name:
                        self._stepper = s
                        logging.info("TMC2130 %s: Stepper Found" % (self.name))
                        break
                if self._stepper is None:
                    logging.info("TMC2130 %s: Stepper NOT Found" % (self.name))
    def _set_default_wave(self):
        # default wave regs from page 79 of TMC2130 datasheet
        msg = "TMC2130: Wave factor on stepper [%s] set to default" \
            % (self.name)
        regs = [
            ("MSLUTSTART", 0x00F70000),
            ("MSLUTSEL", 0xFFFF8056),
            ("MSLUT0", 0xAAAAB554),  # MSLUT0
            ("MSLUT1", 0x4A9554AA),  # MSLUT1
            ("MSLUT2", 0x24492929),  # MSLUT2
            ("MSLUT3", 0x10104222),  # MSLUT3
            ("MSLUT4", 0xFBFFFFFF),  # MSLUT4
            ("MSLUT5", 0xB5BB777D),  # MSLUT5
            ("MSLUT6", 0x49295556),  # MSLUT6
            ("MSLUT7", 0x00404222)   # MSLUT7
        ]
        for reg in regs:
            self.set_register(reg[0], reg[1])
        logging.info(msg)
    def _set_wave(self, fac):
        if fac < TMC_WAVE_FACTOR_MIN:
            fac = 0.0
        elif fac > TMC_WAVE_FACTOR_MAX:
            fac = TMC_WAVE_FACTOR_MAX
        # if fac == 0.:
            # default wave
        #    self._set_default_wave()
        #    return "Wave Set To Default"
        error = None
        vA = 0
        prevA = 0
        delta0 = 0
        delta1 = 1
        w = [1, 1, 1, 1]
        x = [255, 255, 255]
        seg = 0
        bitVal = 0
        deltaA = 0
        reg = 0
        # configure MSLUTSTART
        self.set_register("MSLUTSTART", (TMC_WAVE_AMP << 16))
        for i in range(256):
            if (i & 31) == 0:
                reg = 0
            if fac == 0.:
                vA = int((TMC_WAVE_AMP + 1) * math.sin((2*math.pi*i + math.pi)/1024) + .5) - 1
            else:
                # Prusa corrected wave
                vA = int(
                    TMC_WAVE_AMP * math.pow(math.sin(2*math.pi*i/1024), fac) + .5)
            deltaA = vA - prevA
            prevA = vA
            bitVal = -1
            if deltaA == delta0:
                bitVal = 0
            elif deltaA == delta1:
                bitVal = 1
            else:
                if deltaA < delta0:
                    # switch w bit down
                    bitVal = 0
                    if deltaA == -1:
                        delta0 = -1
                        delta1 = 0
                        w[seg+1] = 0
                    elif deltaA == 0:
                        delta0 = 0
                        delta1 = 1
                        w[seg+1] = 1
                    elif deltaA == 1:
                        delta0 = 1
                        delta1 = 2
                        w[seg+1] = 2
                    else:
                        bitVal = -1
                    if bitVal >= 0:
                        x[seg] = i
                        seg += 1
                elif deltaA > delta1:
                    # switch w bit up
                    bitVal = 1
                    if deltaA == 1:
                        delta0 = 0
                        delta1 = 1
                        w[seg+1] = 1
                    elif deltaA == 2:
                        delta0 = 1
                        delta1 = 2
                        w[seg+1] = 2
                    elif deltaA == 3:
                        delta0 = 2
                        delta1 = 3
                        w[seg+1] = 3
                    else:
                        bitVal = -1
                    if bitVal >= 0:
                        x[seg] = i
                        seg += 1
            if bitVal < 0:
                # delta out of range
                error = "TMC2130: Error setting Sine Wave, Delta Out of Range"
                break
            if seg > 3:  # TODO: should this be greater than 2?
                # segment out of range
                error = "TMC2130: Error setting Sine Wave, Seg Out of Range"
                break
            if bitVal == 1:
                reg |= 0x80000000
            if (i & 31) == 31:
                # configure MSLUT
                self.set_register("MSLUT" + str(((i >> 5) & 7)), reg)
            else:
                reg >>= 1
        # configure MSLUTSEL
        self.set_register(
            "MSLUTSEL", w[0] | (w[1] << 2) | (w[2] << 4) | (w[3] << 6) |
            (x[0] << 8) | (x[1] << 16) | (x[2] << 24))
        if error:
            logging.error(error)
            return error
        else:
            success_msg = "TMC2130: Wave factor on stepper [%s] set to: %f" % \
                          (self.name, fac)
            logging.info(success_msg)
            return success_msg
    cmd_TMC_SET_CURRENT_help = "Set the current applied to a stepper"
    def cmd_TMC_SET_CURRENT(self, params):
        gcode = self.printer.lookup_object('gcode')
        rc = gcode.get_float('RUN', params, above=0., below=2.)
        hc = gcode.get_float('HOLD', params, rc, above=0., below=2.)
        self.tmc2130.set_current_regs(rc, hc)
    cmd_TMC_SET_WAVE_help = "Set wave correction factor for TMC2130 driver"
    def cmd_TMC_SET_WAVE(self, params):
        gcode = self.printer.lookup_object('gcode')
        msg = self._set_wave(gcode.get_float('FACTOR', params))
        gcode.respond_info(msg)
    def cmd_TMC_SET_STEP(self, params):
        force_move = self.printer.lookup_object('force_move')
        move_params = {'STEPPER': self.name}
        gcode = self.printer.lookup_object('gcode')
        if self._stepper is None:
            logging.info(
                "TMC2130 %s: No stepper assigned, cannot step"
                % (self.name))
            gcode.respond_info("Unable to move stepper, unknown stepper ID")
            return
        elif self._stepper.need_motor_enable:
            gcode.respond_info("Cannot Move, motors off")
            return
        max_step = 4 * self.get_microsteps()
        target_step = gcode.get_int('STEP', params, 0, minval=0)
        target_step &= (max_step - 1)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        phase = self.get_phase()
        steps = target_step - phase
        logging.info(
            "TMC_SET_STEP Initial values: target step: %d, phase: %d, steps to move: %d"
            % (target_step, phase, steps))
        # TODO: Accessing a private member is a no-no, but I have no choice.  I need
        # to know the step direction. Maybe I can find another place to get this,
        # perhaps from the config
        direction = 1 if self._stepper.mcu_stepper._invert_dir else -1
        if steps < 0:
            direction *= -1
            steps *= -1
        if steps > (max_step / 2):
            direction *= -1
            steps = max_step - steps
        distance = self._stepper.get_step_dist() * steps * direction
        mcu_pos = self._stepper.get_commanded_position()
        # Move stepper to requested step in sine wave table
        move_params['DISTANCE'] = distance
        move_params['VELOCITY'] = 5
        force_move.cmd_FORCE_MOVE(move_params)
        toolhead.wait_moves()
        self._stepper.set_commanded_position(mcu_pos)
        # Check MSCNT
        phase = self.get_phase()
        if phase != target_step:
            gcode.respond_info("Unable to move to correct step")
            logging.info(
                "TMC2130 %s: TMC_SET_STEP Invalid MSCNT: %d, Target: %d" %
                (self.name, phase, target_step))
        else:
            gcode.respond_info("Correctly moved to step %d:" % target_step)
