# TMC2130 configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import bus

TMC_FREQUENCY=13200000.

Registers = tmc2130_extra.Registers

ReadRegisters = [
    "GCONF", "GSTAT", "IOIN", "TSTEP", "XDIRECT", "MSCNT", "MSCURACT",
    "CHOPCONF", "DRV_STATUS", "PWM_SCALE", "LOST_STEPS",
]

Fields = {}
Fields["GCONF"] = {
    "I_scale_analog": 1<<0, "internal_Rsense": 1<<1, "en_pwm_mode": 1<<2,
    "enc_commutation": 1<<3, "shaft": 1<<4, "diag0_error": 1<<5,
    "diag0_otpw": 1<<6, "diag0_stall": 1<<7, "diag1_stall": 1<<8,
    "diag1_index": 1<<9, "diag1_onstate": 1<<10, "diag1_steps_skipped": 1<<11,
    "diag0_int_pushpull": 1<<12, "diag1_pushpull": 1<<13,
    "small_hysteresis": 1<<14, "stop_enable": 1<<15, "direct_mode": 1<<16,
    "vsense": 1<<17, "vhighfs": 1<<18, "vhighchm": 1<<19, "sync": 0x0f << 20,
    "MRES": 0x0f << 24, "intpol": 1<<28, "dedge": 1<<29, "diss2g": 1<<30
}
Fields["COOLCONF"] = {
    "semin": 0x0f, "seup": 0x03 << 5, "semax": 0x0f << 8, "sedn": 0x03 << 13,
    "seimin": 1<<15, "sgt": 0x7f << 16, "sfilt": 1<<24
}
Fields["DRV_STATUS"] = {
    "SG_RESULT": 0x3ff, "fsactive": 1<<15, "CS_ACTUAL": 0x1f << 16,
    "stallGuard": 1<<24, "ot": 1<<25, "otpw": 1<<26, "s2ga": 1<<27,
    "s2gb": 1<<28, "ola": 1<<29, "olb": 1<<30, "stst": 1<<31
}
Fields["PWMCONF"] = {
    "PWM_AMPL": 0xff, "PWM_GRAD": 0xff << 8, "pwm_freq": 0x03 << 16,
    "pwm_autoscale": 1<<18, "pwm_symmetric": 1<<19, "freewheel": 0x03 << 20
}
Fields["PWM_SCALE"] = { "PWM_SCALE": 0xff }
Fields["LOST_STEPS"] = { "LOST_STEPS": 0xfffff }

FieldFormatters = {
    "I_scale_analog":   (lambda v: "1(ExtVREF)" if v else ""),
    "shaft":            (lambda v: "1(Reverse)" if v else ""),
    "drv_err":          (lambda v: "1(ErrorShutdown!)" if v else ""),
    "uv_cp":            (lambda v: "1(Undervoltage!)" if v else ""),
    "VERSION":          (lambda v: "%#x" % v),
    "CUR_A":            (lambda v: decode_signed_int(v, 9)),
    "CUR_B":            (lambda v: decode_signed_int(v, 9)),
    "MRES":             (lambda v: "%d(%dusteps)" % (v, 0x100 >> v)),
    "otpw":             (lambda v: "1(OvertempWarning!)" if v else ""),
    "ot":               (lambda v: "1(OvertempError!)" if v else ""),
    "s2ga":             (lambda v: "1(ShortToGND_A!)" if v else ""),
    "s2gb":             (lambda v: "1(ShortToGND_B!)" if v else ""),
    "ola":              (lambda v: "1(OpenLoad_A!)" if v else ""),
    "olb":              (lambda v: "1(OpenLoad_B!)" if v else ""),
    "sgt":              (lambda v: decode_signed_int(v, 7)),
}


######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

# Decode two's complement signed integer
def decode_signed_int(val, bits):
    if ((val >> (bits - 1)) & 1):
        return val - (1 << bits)
    return val

class FieldHelper:
    def __init__(self, all_fields, field_formatters={}, registers=None):
        self.all_fields = all_fields
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = {}
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers[reg_name]
        mask = self.all_fields[reg_name][field_name]
        return (reg_value & mask) >> ffs(mask)
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default, config_name=None):
        # Allow a field to be set from the config file
        if config_name is None:
            config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            fval = (value & mask) >> ffs(mask)
            sval = self.field_formatters.get(field_name, str)(fval)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", value, "".join(fields))


######################################################################
# Config reading helpers
######################################################################

def current_bits(current, sense_resistor, vsense_on):
    sense_resistor += 0.020
    vsense = 0.32
    if vsense_on:
        vsense = 0.18
    cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense - 1. + .5)
    return max(0, min(31, cs))

def get_config_current(config):
    vsense = False
    run_current = config.getfloat('run_current', above=0., maxval=2.)
    hold_current = config.getfloat('hold_current', run_current,
                                   above=0., maxval=2.)
    sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
    irun = current_bits(run_current, sense_resistor, vsense)
    ihold = current_bits(hold_current, sense_resistor, vsense)
    if irun < 16 and ihold < 16:
        vsense = True
        irun = current_bits(run_current, sense_resistor, vsense)
        ihold = current_bits(hold_current, sense_resistor, vsense)
    return vsense, irun, ihold

def get_config_microsteps(config):
    steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
             '8': 5, '4': 6, '2': 7, '1': 8}
    return config.getchoice('microsteps', steps)

def get_config_stealthchop(config, tmc_freq):
    mres = get_config_microsteps(config)
    velocity = config.getfloat('stealthchop_threshold', 0., minval=0.)
    if not velocity:
        return mres, False, 0
    stepper_name = " ".join(config.get_name().split()[1:])
    stepper_config = config.getsection(stepper_name)
    step_dist = stepper_config.getfloat('step_distance')
    step_dist_256 = step_dist / (1 << mres)
    threshold = int(tmc_freq * step_dist_256 / velocity + .5)
    return mres, True, max(0, min(0xfffff, threshold))


######################################################################
# TMC2130 printer object
######################################################################

class TMC2130:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=4000000)
        # Allow virtual endstop to be created
        self.diag1_pin = config.get('diag1_pin', None)
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("tmc2130_" + self.name, self)
        gcode = self.printer.lookup_object("gcode")
        step_dist = config.getsection(self.name).getfloat('step_distance')
        # Get config for initial driver settings
        self.run_current = config.getfloat('run_current', above=0., maxval=2.)
        self.sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,}
        self.mres = config.getchoice('microsteps', steps)
        self.step_dist_256 = step_dist / (1 << self.mres)
        interpolate = config.getboolean('interpolate', True)
        sc_velocity = config.getfloat('stealthchop_threshold', None, minval=0.)
        self.iholddelay = config.getint(
            'driver_IHOLDDELAY', 8, minval=0, maxval=15)
        blank_time_select = config.getint('driver_BLANK_TIME_SELECT', 1,
                                          minval=0, maxval=3)
        hend = config.getint('driver_HEND', 7, minval=0, maxval=15)
        hstrt = config.getint('driver_HSTRT', 0, minval=0, maxval=7)
        sgt = config.getint('driver_SGT', 0, minval=-64, maxval=63) & 0x7f
        pwm_scale = config.getboolean('driver_PWM_AUTOSCALE', True)
        pwm_freq = config.getint('driver_PWM_FREQ', 1, minval=0, maxval=3)
        pwm_grad = config.getint('driver_PWM_GRAD', 4, minval=0, maxval=255)
        pwm_ampl = config.getint('driver_PWM_AMPL', 128, minval=0, maxval=255)

        # new CHOPCONF options
        ch_modes = {'SpreadCycle': 0, 'ConstantOff': 1}
        chopper_mode = config.getchoice(
            'chopper_mode', ch_modes, "SpreadCycle")
        rndtf = config.getboolean('driver_RNDTF', False)
        tfd = config.getint('driver_TFD', 0, minval=0, maxval=15)
        offset = config.getint('driver_OFFSET', 0, minval=0, maxval=15)
        disfdcc = config.getboolean('driver_DISFDCC', False)
        vhighfs = config.getboolean('driver_VHIGHFS', False)
        vhighchm = config.getboolean('driver_VHIGHCHM', False)
        sync = config.getint('driver_SYNC', 0, minval=0, maxval=15)
        dedge = config.getboolean('driver_DEDGE', False)
        diss2g = config.getboolean('driver_DISS2G', False)

        # new COOLCONF options
        semin = config.getint('driver_SEMIN', 0, minval=0, maxval=15)
        seup_choices = {'1': 0, '2': 1, '4': 2, '8': 3}
        seup = config.getchoice('driver_SEUP', seup_choices, '1')
        semax = config.getint('driver_SEMAX', 0, minval=0, maxval=15)
        sedn_choices = {'32': 0, '8': 1, '2': 2, '1': 3}
        sedn = config.getchoice('driver_SEDN', sedn_choices, '32')
        seimin = config.getint('driver_SEIMIN', 0, minval=0, maxval=1)
        sfilt = config.getboolean('driver_SFILT', False)

        # other new driver options
        tc_velocity = config.getint(
            'coolstep_threshold', 0, minval=0)
        self.tcoolthrs = self.velocity_to_clock(tc_velocity)
        thigh_velocity = config.getint(
            'thigh_threshold', 0, minval=0)
        th_threshold = self.velocity_to_clock(thigh_velocity)
        self.vsense = config.getboolean('driver_VSENSE', None)

        # Configure registers
        self.reg_GCONF = (sc_velocity is not None) << 2
        if chopper_mode == 0:
            # spreadcycle
            cm_bits = (hstrt << 4) | (hend << 7)
        else:
            # constant off with fast decay
            cm_bits = (((tfd & 0x07) << 4) | (offset << 7) |
                       ((tfd & 0x08) << 11) | (disfdcc << 12))
        self.reg_CHOP_CONF = (toff | cm_bits | (rndtf << 13) |
                              (chopper_mode << 14) |
                              (blank_time_select << 15) | (vhighfs << 18) |
                              (vhighchm << 19) | (sync << 20) |
                              (self.mres << 24) | (interpolate << 28) |
                              (dedge << 29) | (diss2g << 30))
        self.reg_COOL_CONF = (semin | (seup << 5) | (semax << 8) |
                              (sedn << 13) | (seimin << 15) | (sgt << 16) |
                              (sfilt << 24))
        self.reg_PWM_CONF = (pwm_ampl | (pwm_grad << 8) | (pwm_freq << 16) |
                             (pwm_scale << 18))
        self.set_register("GCONF", self.reg_GCONF)
        self.set_current_regs(self.run_current, self.hold_current)
        self.set_register("TPOWERDOWN", tpowerdown)
        self.set_register("TPWMTHRS", max(0, min(0xfffff, sc_threshold)))
        # TODO: need to use velocity to clock for toolthrs and high?
        self.set_register("TCOOLTHRS", max(0, min(0xfffff, self.tcoolthrs)))
        self.set_register("THIGH", max(0, min(0xfffff, th_threshold)))
        self.set_register("COOLCONF", self.reg_COOL_CONF)
        self.set_register("PWMCONF", self.reg_PWM_CONF)
        self.extras = tmc2130_extra.TMC2130_EXTRA(config, self)
    def set_current_regs(self, run_current, hold_current):
        def get_bits(rc, hc, vs):
            run = self.current_bits(rc, self.sense_resistor, vs)
            hold = self.current_bits(hc, self.sense_resistor, vs)
            return run, hold
        if self.vsense is None:
            # Auto Calculate Vsense
            vsense = False
            irun, ihold = get_bits(run_current, hold_current, vsense)
            if irun < 16 and ihold < 16:
                vsense = True
                irun, ihold = get_bits(run_current, hold_current, vsense)
        else:
            vsense = self.vsense
            irun, ihold = get_bits(run_current, hold_current, self.vsense)
        self.reg_CHOP_CONF &= ~(1 << 17)
        self.reg_CHOP_CONF |= (vsense << 17)
        ihold_irun = ihold | (irun << 8) | (self.iholddelay << 16)
        self.set_register("CHOPCONF", self.reg_CHOP_CONF)
        self.set_register("IHOLD_IRUN", ihold_irun)
    def current_bits(self, current, sense_resistor, vsense_on):
        sense_resistor += 0.020
        vsense = 0.32
        if vsense_on:
            vsense = 0.18
        cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense
                 - 1. + .5)
        return max(0, min(31, cs))
    def velocity_to_clock(self, velocity):
        if not velocity:
            return 0
        return int(TMC_FREQUENCY * self.step_dist_256 / velocity + .5)
=======
        # Setup basic register values
        self.regs = collections.OrderedDict()
        self.fields = FieldHelper(Fields, FieldFormatters, self.regs)
        vsense, irun, ihold = get_config_current(config)
        self.fields.set_field("vsense", vsense)
        self.fields.set_field("IHOLD", ihold)
        self.fields.set_field("IRUN", irun)
        mres, en_pwm, thresh = get_config_stealthchop(config, TMC_FREQUENCY)
        self.fields.set_field("MRES", mres)
        self.fields.set_field("en_pwm_mode", en_pwm)
        self.fields.set_field("TPWMTHRS", thresh)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        set_config_field(config, "toff", 4)
        set_config_field(config, "hstrt", 0)
        set_config_field(config, "hend", 7)
        set_config_field(config, "TBL", 1, "driver_BLANK_TIME_SELECT")
        set_config_field(config, "intpol", True, "interpolate")
        set_config_field(config, "IHOLDDELAY", 8)
        set_config_field(config, "TPOWERDOWN", 0)
        set_config_field(config, "PWM_AMPL", 128)
        set_config_field(config, "PWM_GRAD", 4)
        set_config_field(config, "pwm_freq", 1)
        set_config_field(config, "pwm_autoscale", True)
        sgt = config.getint('driver_SGT', 0, minval=-64, maxval=63) & 0x7f
        self.fields.set_field("sgt", sgt)
        # Send registers
        for reg_name, val in self.regs.items():
            self.set_register(reg_name, val)
>>>>>>> upstream/master
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise pins.error("tmc2130 virtual endstop only useful as endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert tmc2130 virtual endstop")
        return TMC2130VirtualEndstop(self)
    def get_register(self, reg_name):
        reg = Registers[reg_name]
        self.spi.spi_send([reg, 0x00, 0x00, 0x00, 0x00])
        params = self.spi.spi_transfer([reg, 0x00, 0x00, 0x00, 0x00])
        pr = bytearray(params['response'])
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]
    def set_register(self, reg_name, val):
        reg = Registers[reg_name]
        data = [(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
                (val >> 8) & 0xff, val & 0xff]
        self.spi.spi_send(data)
    def get_microsteps(self):
        return 256 >> self.fields.get_field("MRES")
    def get_phase(self):
        mscnt = self.fields.get_field("MSCNT", self.get_register("MSCNT"))
        return mscnt >> self.fields.get_field("MRES")
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        logging.info("DUMP_TMC %s", self.name)
        for reg_name in ReadRegisters:
            val = self.get_register(reg_name)
            msg = "%-15s %08x" % (reg_name + ":", val)
            logging.info(msg)
            gcode.respond_info(msg)

# Endstop wrapper that enables tmc2130 "sensorless homing"
class TMC2130VirtualEndstop:
    def __init__(self, tmc2130):
        self.tmc2130 = tmc2130
        if tmc2130.diag1_pin is None:
            raise pins.error("tmc2130 virtual endstop requires diag1_pin")
        ppins = tmc2130.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', tmc2130.diag1_pin)
        if self.mcu_endstop.get_mcu() is not tmc2130.spi.get_mcu():
            raise pins.error("tmc2130 virtual endstop must be on same mcu")
        self.en_pwm = tmc2130.fields.get_field("en_pwm_mode")
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        self.query_endstop_wait = self.mcu_endstop.query_endstop_wait
        self.TimeoutError = self.mcu_endstop.TimeoutError
    def home_prepare(self):
        self.tmc2130.fields.set_field("en_pwm_mode", 0)
        self.tmc2130.fields.set_field("diag1_stall", 1)
        self.tmc2130.set_register("GCONF", self.tmc2130.regs['GCONF'])
        self.tmc2130.set_register("TCOOLTHRS", 0xfffff)
        self.mcu_endstop.home_prepare()
    def home_finalize(self):
        self.tmc2130.fields.set_field("en_pwm_mode", self.en_pwm)
        self.tmc2130.fields.set_field("diag1_stall", 0)
        self.tmc2130.set_register("GCONF", self.tmc2130.regs['GCONF'])
        self.tmc2130.set_register("TCOOLTHRS", 0)
        self.mcu_endstop.home_finalize()

def load_config_prefix(config):
    return TMC2130(config)
