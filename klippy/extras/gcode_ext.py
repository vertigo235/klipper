# Extended Gcode Commands
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class GCodeExt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean('enabled', True)
        self.config_extruder_accel = None
        if self.enabled:
            self.gcode = self.printer.lookup_object('gcode')
            self.gcode.register_command('M204', self.cmd_M204,
                                        desc=self.cmd_M204_help)
    cmd_M204_help = "Set printer acceleration limit"
    def printer_state(self, state):
        if state == 'ready':
            if self.enabled and (self.config_extruder_accel == None):
                extruder = self.printer.lookup_object('extruder0')
                self.config_extruder_accel = extruder.max_e_accel
    def cmd_M204(self, params):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        # Check for "old style" Marlin pre-2015 change to Printing Acceleration
        max_accel = self.gcode.get_float('S', params, None, above=0.,
                                         maxval=toolhead.config_max_accel)
        # Check new Style Marlin M204 (P/R/T)
        if not max_accel:
            max_accel = self.gcode.get_float('P', params, None, above=0.,
                                              maxval=toolhead.config_max_accel)
        # If M204 did not include acceleration for a print move, check for
        # a Travel move
        if not max_accel:
            max_accel = self.gcode.get_float('T', params, None, above=0., 
                                             maxval=toolhead.config_max_accel)
        max_retract_accel = self.gcode.get_float('R', params, None, above=0.,
                                                 maxval=self.config_extruder_accel)
        if max_accel:
            toolhead.max_accel = max_accel
            # when changing acceleration, use Klipper's default for accel_to_decel
            toolhead.max_accel_to_decel = .5 * toolhead.max_accel
        extruder = self.printer.lookup_object('extruder0')
        if max_retract_accel:    
            extruder.max_e_accel = max_retract_accel
        self.gcode.respond_info("M204 Set Max Accel: Print: %.6f, Retract: %.6f, Travel: %.6f" 
                            % (toolhead.max_accel, extruder.max_e_accel, toolhead.max_accel))

def load_config(config):
    return GCodeExt(config)