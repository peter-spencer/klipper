# G Code Support for Hexapod Kinematics
#
# Copyright (C) 2026  Peter Spencer <peter@quantized.art>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, inspect, ast

class HexapodGCode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        handlers = [func for func in inspect.getmembers(self)
                    if func[0].startswith('gcode_')]

        for handler in handlers:
            func = handler[1]
            cmd = handler[0].removeprefix('gcode_')
            desc = inspect.getdoc(handler[1])
            self.gcode.register_command(cmd, func, False, desc)

    def gcode_GET_HEXAPOD_POSITION(self, gcmd):
        """Returns the 6D position of the printer."""

        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        kin = toolhead.get_kinematics()
        steppers = kin.get_steppers()

        cinfo = [(s.get_name(), s.get_commanded_position()) for s in steppers]
        kinfo = zip("XYZABC", kin.calc_position(dict(cinfo)))

        pos = " ".join(["%s:%.6f" % (a, v) for a, v in kinfo])

        gcmd.respond_info(pos)

    # def gcode_SET_ROTATION_MOVE(self, gcmd):
    #     """Setup the rotational part of a 6D printer move."""
    #     pass

def load_config(config):
    return HexapodGCode(config)