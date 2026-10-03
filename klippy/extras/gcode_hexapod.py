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

        axis_map = {'X':0, 'Y': 1, 'Z': 2, 'E': 3}
        extra_axes = toolhead.get_extra_axes()
        for index, ea in enumerate(extra_axes):
            if ea is None:
                continue
            gcode_id = ea.get_axis_gcode_id()
            if (gcode_id is None or len(gcode_id) != 1 or not gcode_id.isupper()
                or gcode_id in axis_map or gcode_id in "FN"):
                continue
            axis_map[gcode_id] = index

        tpos_all = toolhead.get_position()
        tpos_names = axis_map.keys()
        tpos_values = [tpos_all[i] for i in axis_map.values()]

        toolhead_pos = " ".join(["%s:%.6f" % (a, v) for a, v in zip(
                    tpos_names, tpos_values)])

        gcmd.respond_info("kinematic: %s\ntoolhead: %s" % (pos, toolhead_pos))

    # def gcode_SET_ROTATION_MOVE(self, gcmd):
    #     """Setup the rotational part of a 6D printer move."""
    #     pass

def load_config(config):
    return HexapodGCode(config)