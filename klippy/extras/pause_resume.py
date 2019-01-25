# Pause/Resume functionality with position capture/restore
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class PauseResume:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.recover_velocity = config.getfloat('recover_velocity', 50.)
        self.captured_position = None
        self.captured_speed = 0.
        self.toolhead = self.v_sd = None
        self.sd_paused = False
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command("PAUSE", self.cmd_PAUSE)
        self.gcode.register_command("RESUME", self.cmd_RESUME)
    def handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
    def cmd_PAUSE(self, params):
        if self.v_sd is not None and self.v_sd.is_active():
            # Printing from virtual sd, run pause command
            self.sd_paused = True
            self.v_sd.cmd_M25({})
        else:
            self.sd_paused = False
            self.gcode.respond_info("action:pause")
        self.toolhead.wait_moves()
        self.captured_position = self.toolhead.get_position()
        reactor = self.printer.get_reactor()
        gc_status = self.gcode.get_status(reactor.monotonic())
        self.captured_speed = gc_status['speed']
    def cmd_RESUME(self, params):
        velocity = self.gcode.get_float(
            'VELOCITY', params, self.recover_velocity)
        if self.captured_position is not None:
            # restore previous xyz position
            cur_pos = self.toolhead.get_position()
            self.captured_position[3] = cur_pos[3]
            self.toolhead.move(self.captured_position, velocity)
            self.toolhead.get_last_move_time()
            self.gcode.reset_last_position()
            # restore previous speed
            self.gcode.run_script_from_command(
                "G1 F%.6f" % (self.captured_speed))
        self.captured_position = None
        if self.sd_paused:
            # Printing from virtual sd, run pause command
            self.v_sd.cmd_M24({})
        else:
            self.gcode.respond_info("action:resume")

def load_config(config):
    return PauseResume(config)
