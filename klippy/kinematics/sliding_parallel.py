# Code for handling the kinematics of a Gough-Stewart mechanism driven by linear slides
#
# Copyright (C) 2026 Peter Spencer <peter@quantized.art>
# Initially derived from delta.py by Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

class SlidingParallelKinematics:
    def __init__(self, toolhead, config):
        config.error("Initialising Sliding Parallel Kinematics Class.")
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abcdef']
        # rail_a = stepper.LookupMultiRail(
        #     stepper_configs[0], need_position_minmax = False)
        # a_endstop = rail_a.get_homing_info().position_endstop
        # rail_b = stepper.LookupMultiRail(
        #     stepper_configs[1], need_position_minmax = False,
        #     default_position_endstop=a_endstop)
        # rail_c = stepper.LookupMultiRail(
        #     stepper_configs[2], need_position_minmax = False,
        #     default_position_endstop=a_endstop)
        # rail_d = stepper.LookupMultiRail(
        #     stepper_configs[3], need_position_minmax = False,
        #     default_position_endstop=a_endstop)
        # rail_e = stepper.LookupMultiRail(
        #     stepper_configs[4], need_position_minmax = False,
        #     default_position_endstop=a_endstop)
        # rail_f = stepper.LookupMultiRail(
        #     stepper_configs[5], need_position_minmax = False,
        #     default_position_endstop=a_endstop)
        # self.rails = [rail_a, rail_b, rail_c,rail_d, rail_e, rail_f]
        a_endstop = config.getsection('stepper_a').getfloat('position_endstop', None)
        self.rails = [stepper.LookupMultiRail(a, need_position_minmax = False,
            default_position_endstop=a_endstop) for a in stepper_configs]

        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                            above=0., maxval=self.max_accel)
        
        # Read radius and arm lengths
        self.radius = radius = config.getfloat('delta_radius', above=0.)
        print_radius = config.getfloat('print_radius', radius, above=0.)
        arm_length_a = stepper_configs[0].getfloat('arm_length', above=radius)
        self.arm_lengths = arm_lengths = [
            sconfig.getfloat('arm_length', arm_length_a, above=radius)
            for sconfig in stepper_configs]
        self.arm2 = [arm**2 for arm in arm_lengths]
        self.abs_endstops = [(rail.get_homing_info().position_endstop
                                + math.sqrt(arm2 - radius**2))
                                for rail, arm2 in zip(self.rails, self.arm2)]
        
        # Determine tower locations in cartesian space
        # First get simple or default spacings
        a = config.getfloat('tower_pair_start', 210.)                 # Angle of the centre of the first pair (angle in degrees)
        b = config.getfloat('tower_pair_spacing', 120., above=0.)     # Spacing between centre of each pair (angle in degrees)
        c = config.getfloat('tower_pair_gap', 10., above=0.) / 2      # Gap between each pair (angle in degrees)
        default_tower_angles = [ (a-c)%360.0, (a+c)%360.0, (a+b-c)%360.0, (a+b+c)%360.0, (a+2*b-c)%360.0, (a+2*b+c)%360.0 ]

        # Now calculate the specific angles and positions of the towers
        self.angles = [sconfig.getfloat('angle', angle)
                       for sconfig, angle in zip(stepper_configs, default_tower_angles)]
        self.towers = [(math.cos(math.radians(angle)) * radius,
                        math.sin(math.radians(angle)) * radius)
                       for angle in self.angles]

        logging.info(
                    "Axis tower angles: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f degrees"
                    % (self.angles[0],self.angles[1],self.angles[2],self.angles[3],self.angles[4],self.angles[5]))

        # Determine joint locations on the effector
        effector_radius = config.getfloat('effector_radius', above=0.)   # Radius of the circle of joints on the effector (distance in mm)
        effector_z = config.getfloat('effector_z', 0.)                   # Height offset of the joints on the effector (displacement in mm)
        a = config.getfloat('effector_pair_spacing', 150.)               # Angle of the centre of the first pair (angle in degrees)
        b = config.getfloat('effector_pair_spacing', 120., above=0.)     # Spacing between centre of each pair (angle in degrees)
        c = config.getfloat('effector_pair_gap', 10., above=0.) * .5     # Gap between each pair (angle in degrees)
        default_joint_angles = [ (a+c)%360.0, (a+b-c)%360.0, (a+b+c)%360.0, (a+2*b-c)%360.0, (a+2*b+c)%360.0, (a-c)%360.0 ]

        # Now calculate the specific angles and positions of the towers
        joint_angles = [sconfig.getfloat('effector_angle', angle)
                        for sconfig, angle in zip(stepper_configs, default_joint_angles)]
        self.joints = [(math.cos(math.radians(angle)) * effector_radius,
                        math.sin(math.radians(angle)) * effector_radius,
                        effector_z)
                        for angle in joint_angles]

        logging.info(
                    "Effector joint angles: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f degrees"
                    % (joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5]))


        # Setup the iterative solver (for converting XYZ move into stepper movements)
        for r, a, t, j in zip(self.rails, self.arm2, self.towers, self.joints):
            r.setup_itersolve('sliding_parallel_stepper_alloc', a, t[0], t[1], j[0], j[1], j[2])
        
        # Setup trapezoidal generator / look-ahead queue
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple(
            self._actuator_to_cartesian(self.abs_endstops))
        self.max_z = min([rail.get_homing_info().position_endstop
                          for rail in self.rails])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        self.limit_z = min([ep - arm
                            for ep, arm in zip(self.abs_endstops, arm_lengths)])
        self.min_arm_length = min_arm_length = min(arm_lengths)
        self.min_arm2 = min_arm_length**2
        logging.info(
            "Delta max build height %.2fmm (radius tapered above %.2fmm)"
            % (self.max_z, self.limit_z))
        
        # Find the point where an XY move could result in excessive
        # tower movement
        half_min_step_dist = min([r.get_steppers()[0].get_step_dist()
                                  for r in self.rails]) * .5
        min_arm_length = min(arm_lengths)
        def ratio_to_xy(ratio):
            return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.)
                                      - half_min_step_dist**2)
                    + half_min_step_dist - radius)
        self.slow_xy2 = ratio_to_xy(SLOW_RATIO)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO)**2
        self.max_xy2 = min(print_radius, min_arm_length - radius,
                           ratio_to_xy(4. * SLOW_RATIO))**2
        max_xy = math.sqrt(self.max_xy2)
        logging.info("Delta max build radius %.2fmm (moves slowed past %.2fmm"
                     " and %.2fmm)"
                     % (max_xy, math.sqrt(self.slow_xy2),
                        math.sqrt(self.very_slow_xy2)))

        # Set minium and maximum axes positions
        self.axes_min = toolhead.Coord((-max_xy, -max_xy, self.min_z))
        self.axes_max = toolhead.Coord((max_xy, max_xy, self.max_z))

        # Zero the printer position
        self.set_position([0., 0., 0.], "")


        
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    # Forward kinematics (work out XYZ position from the stepper positions, requires numeric solution)
    def _actuator_to_cartesian(self, spos):
        # sphere_coords = [(t[0], t[1], sp) for t, sp in zip(self.towers, spos)]
        sphere_coords = [(t[0], t[1], sp) for t, sp in zip([self.towers[0],self.towers[2],self.towers[4]], [spos[0],spos[2],spos[4],])]
        return mathutil.trilateration(sphere_coords, self.arm2)
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    # Set the stepper motor positions
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if homing_axes == "xyz":
            self.need_home = False

    # Homing the printer
    def clear_homing_state(self, clear_axes):
        # Clearing homing state for each axis individually is not implemented
        if clear_axes:
            self.limit_xy2 = -1
            self.need_home = True
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    # Check that a proposed move will be possible
    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            allowed_radius = self.radius - math.sqrt(
                self.min_arm2 - (self.min_arm_length - above_z_limit)**2
            )
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)

    # Return status information
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'cone_start_z': self.limit_z,
        }

    # Get the printer calibration
    def get_calibration(self):
        endstops = [rail.get_homing_info().position_endstop
                    for rail in self.rails]
        stepdists = [rail.get_steppers()[0].get_step_dist()
                     for rail in self.rails]
        return SlidingParallelCalibration(self.radius, self.angles, self.arm_lengths,
                                endstops, stepdists)

# Delta parameter calibration for DELTA_CALIBRATE tool
class SlidingParallelCalibration:
    def __init__(self, radius, angles, arms, endstops, stepdists):
        self.radius = radius
        self.angles = angles
        self.arms = arms
        self.endstops = endstops
        self.stepdists = stepdists
        # Calculate the XY cartesian coordinates of the delta towers
        radian_angles = [math.radians(a) for a in angles]
        self.towers = [(math.cos(a) * radius, math.sin(a) * radius)
                       for a in radian_angles]
        # Calculate the absolute Z height of each tower endstop
        radius2 = radius**2
        self.abs_endstops = [e + math.sqrt(a**2 - radius2)
                             for e, a in zip(endstops, arms)]
    def coordinate_descent_params(self, is_extended):
        # Determine adjustment parameters (for use with coordinate_descent)
        adj_params = ('radius', 'angle_a', 'angle_b',
                      'endstop_a', 'endstop_b', 'endstop_c')
        if is_extended:
            adj_params += ('arm_a', 'arm_b', 'arm_c')
        params = { 'radius': self.radius }
        for i, axis in enumerate('abc'):
            params['angle_'+axis] = self.angles[i]
            params['arm_'+axis] = self.arms[i]
            params['endstop_'+axis] = self.endstops[i]
            params['stepdist_'+axis] = self.stepdists[i]
        return adj_params, params
    def new_calibration(self, params):
        # Create a new calibration object from coordinate_descent params
        radius = params['radius']
        angles = [params['angle_'+a] for a in 'abc']
        arms = [params['arm_'+a] for a in 'abc']
        endstops = [params['endstop_'+a] for a in 'abc']
        stepdists = [params['stepdist_'+a] for a in 'abc']
        return SlidingParallelCalibration(radius, angles, arms, endstops, stepdists)
    def get_position_from_stable(self, stable_position):
        # Return cartesian coordinates for the given stable_position
        sphere_coords = [
            (t[0], t[1], es - sp * sd)
            for sd, t, es, sp in zip(self.stepdists, self.towers,
                                     self.abs_endstops, stable_position) ]
        return mathutil.trilateration(sphere_coords, [a**2 for a in self.arms])
    def calc_stable_position(self, coord):
        # Return a stable_position from a cartesian coordinate
        steppos = [
            math.sqrt(a**2 - (t[0]-coord[0])**2 - (t[1]-coord[1])**2) + coord[2]
            for t, a in zip(self.towers, self.arms) ]
        return [(ep - sp) / sd
                for sd, ep, sp in zip(self.stepdists,
                                      self.abs_endstops, steppos)]
    def save_state(self, configfile):
        # Save the current parameters (for use with SAVE_CONFIG)
        configfile.set('printer', 'delta_radius', "%.6f" % (self.radius,))
        for i, axis in enumerate('abc'):
            configfile.set('stepper_'+axis, 'angle', "%.6f" % (self.angles[i],))
            configfile.set('stepper_'+axis, 'arm_length',
                           "%.6f" % (self.arms[i],))
            configfile.set('stepper_'+axis, 'position_endstop',
                           "%.6f" % (self.endstops[i],))
        gcode = configfile.get_printer().lookup_object("gcode")
        gcode.respond_info(
            "stepper_a: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_b: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_c: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "delta_radius: %.6f"
            % (self.endstops[0], self.angles[0], self.arms[0],
               self.endstops[1], self.angles[1], self.arms[1],
               self.endstops[2], self.angles[2], self.arms[2],
               self.radius))

def load_kinematics(toolhead, config):
    return SlidingParallelKinematics(toolhead, config)