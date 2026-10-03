# Code for handling the kinematics of a Gough-Stewart mechanism driven by linear slides
#
# Copyright (C) 2026 Peter Spencer <peter@quantized.art>
# Initially derived from delta.py by Kevin O'Connor
#
# TODO:
#  - Add a safety net for the Newton solver to stop it trying mathematically impossible solutions, or going out of bounds at all
#  - check_move() to be improved
#  - Is it worthwhile to have a more rigorous stepper speed check and movement slowing?
#  - IMPLEMENT STATIC ROTATIONS OF THE EFFECTOR
#  - longer term: DYNAMIC ROTATIONS (extra axes, iterative solver access?, accessible print areas and checks)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import chelper, math, logging
import stepper, mathutil

# delay import until configuration time
np = None
scipy = None

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

class HexapodKinematics:
    def __init__(self, toolhead, config):
        try:
            global np
            import numpy as np
        except:
            raise config.error("Hexapod kinematics requires the NumPy module")
        try:
            global scipy
            import scipy
        except:
            raise config.error("Hexapod kinematics requires the SciPy module")

        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abcdef']

        # Remember the toolhead and config (DOES DOING THIS BREAK ANYTHING LATER????)
        self.toolhead = toolhead
        self.config = config
        self.stepper_configs = stepper_configs

        # Load the first rail, stepper_a
        rail_a = stepper.LookupMultiRail(stepper_configs[0], need_position_minmax = False)

        # There must be an endstop specified for at least this first rail,
        # this will be the default endstop for the other rails.
        default_endstop = rail_a.get_homing_info().position_endstop

        # Load the rest of the rails
        self.rails = [rail_a] + [stepper.LookupMultiRail(stepper_config, need_position_minmax = False,
            default_position_endstop=default_endstop) for stepper_config in stepper_configs[1:]]

        # Setup max velocity
        self.max_velocity, self.max_accel = self.toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                            above=0., maxval=self.max_accel)
        
        # Read radius and arm lengths
        self.radius = radius = config.getfloat('delta_radius', above=0.)
        print_radius = config.getfloat('print_radius', radius, above=0.)
        arm_length_a = stepper_configs[0].getfloat('arm_length', above=radius)
        self.arm_lengths = [sconfig.getfloat('arm_length', arm_length_a, above=radius)
                            for sconfig in stepper_configs]

        # Store the squared length of the printer's arms to save on calculation time
        self.arm2 = np.square(self.arm_lengths)
        
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

        self._calculate_effector_joints()

        # Calculate the absolute position of each endstop. The normal endstop is the
        # height of the nozzle above the heated bed (in mm), the absolute endstop is
        # that stepper's position along its rail when homed.
        self.endstops = [rail.get_homing_info().position_endstop for rail in self.rails]
        self.abs_endstops = [self._cartesian_to_actuator((0., 0., endstop, 0., 0., 0.), arm2, tower, joint)
                             for endstop, arm2, tower, joint in zip(self.endstops, self.arm2, self.towers, self.joints)]

        logging.info(
                    "Absolute endstop positions: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f mm"
                    % (self.abs_endstops[0],self.abs_endstops[1],self.abs_endstops[2],self.abs_endstops[3],self.abs_endstops[4],self.abs_endstops[5]))

        self._setup_iterative_solver()

        # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        self.max_z = min(self.endstops)
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        self.home_position = tuple(self._actuator_to_cartesian(self.abs_endstops))

        # Calculate the highest Z where the full range of XY motion is possible.
        # In a linear delta design, this is when the steppers are at the top of
        # their range, and an arm is directly downwards.
        # For the Gough/Stewart/Hexapod robot, it is a bit more complex:
        #   - The arms are not vertically downward (I think, not sure) at the limit?
        #   - Probably still a reasonable approximation for a limit - CHECK THIS!
        #   - Need calculation with effector level, and accounting for rotations?
        self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, self.arm_lengths)])
        
        logging.info(
            "Hex max build height %.2fmm (radius tapered above %.2fmm)"
            % (self.max_z, self.limit_z))
        
        # Get the shortest arm's length and squared-length.
        self.min_arm_length = min_arm_length = min(self.arm_lengths)
        self.min_arm2 = min_arm_length**2
        
        # Find the point where an XY move could result in excessive
        # tower movement
        half_min_step_dist = min([r.get_steppers()[0].get_step_dist()
                                  for r in self.rails]) * .5

        def ratio_to_xy(ratio):
            return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.)
                                      - half_min_step_dist**2)
                    + half_min_step_dist - radius)

        self.slow_xy2 = ratio_to_xy(SLOW_RATIO)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO)**2
        self.max_xy2 = min(print_radius, min_arm_length - radius,
                           ratio_to_xy(4. * SLOW_RATIO))**2
        max_xy = math.sqrt(self.max_xy2)

        logging.info("Hex max build radius %.2fmm (moves slowed past %.2fmm"
                     " and %.2fmm)"
                     % (max_xy, math.sqrt(self.slow_xy2),
                        math.sqrt(self.very_slow_xy2)))

        # Set minium and maximum axes positions to define allowed build volume
        self.axes_min = self.toolhead.Coord((-max_xy, -max_xy, self.min_z))
        self.axes_max = self.toolhead.Coord((max_xy, max_xy, self.max_z))

        # Why was this here? What did it do?
        # self.set_position([0., 0., 0.], "")

        # Setup extra axes for rotation control via G1 G-code commands
        self.rotation_axes = [ HexapodRotationAxis(self, id) for id in 'ABC' ]

        # Wait for everything to load
        config.get_printer().register_event_handler("klippy:mcu_identify", self._add_extra_axes)

    # Add extra axes to the Toolhead for control over 
    def _add_extra_axes(self):
        for rotation in self.rotation_axes:
            self.toolhead.add_extra_axis(rotation, rotation.commanded_pos)
            
    def _setup_iterative_solver(self):
        ffi_main, ffi_lib = chelper.get_ffi()

        # Setup the iterative solver (for converting XYZ move into stepper movements)
        for r, a, t, es, j in zip(self.rails, self.arm2, self.towers, self.abs_endstops, self.joints):
            r.setup_itersolve('hexapod_stepper_alloc', a, t[0], t[1], es, j[0], j[1], j[2])
            # b = ffi_main.new("struct coord", [t[0], t[1], es])
            # c = ffi_main.new("struct coord", list(j))
            # r.setup_itersolve('hexapod_stepper_alloc', a, b, c)
        
        # Setup trapezoidal generator / look-ahead queue
        for s in self.get_steppers():
            s.set_trapq(self.toolhead.get_trapq())

    def _calculate_effector_joints(self, offset_angle = 0):
        self.offset_angle = offset_angle

        # Determine joint locations on the effector
        effector_radius = self.config.getfloat('effector_radius', above=0.)   # Radius of the circle of joints on the effector (distance in mm)
        effector_z = self.config.getfloat('effector_z', 0.)                   # Height offset of the joints on the effector (displacement in mm)
        a = self.config.getfloat('effector_pair_start', 150.) + offset_angle  # Angle of the centre of the first pair (angle in degrees)
        b = self.config.getfloat('effector_pair_spacing', 120., above=0.)     # Spacing between centre of each pair (angle in degrees)
        c = self.config.getfloat('effector_pair_gap', 10., above=0.) * .5     # Gap between each pair (angle in degrees)
        default_joint_angles = [ (a+c)%360.0, (a+b-c)%360.0, (a+b+c)%360.0, (a+2*b-c)%360.0, (a+2*b+c)%360.0, (a-c)%360.0 ]

        # Now calculate the specific angles and positions of the towers
        joint_angles = [sconfig.getfloat('effector_angle', angle)
                        for sconfig, angle in zip(self.stepper_configs, default_joint_angles)]
        self.joints = [(math.cos(math.radians(angle)) * effector_radius,
                        math.sin(math.radians(angle)) * effector_radius,
                        effector_z)
                        for angle in joint_angles]

        logging.info(
                    "Effector joint angles: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f degrees"
                    % (joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5]))

    # Get the stepper motor positions    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    # Set the stepper motor positions
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if homing_axes == "xyz":
            self.need_home = False


    ######  Inverse Kinematics  ######

    # Return a stepper position for the given coordinates, arm length,
    # tower position, and joint position at the specified coordinates
    def _cartesian_to_actuator(self, coordinates, arm2, tower, joint):
        # 2D position without orientation
        tx, ty = tower

        # 3D positions without orientation
        x, y, z, a, b, c = coordinates

        r = scipy.spatial.transform.Rotation.from_euler('xyz', [a,b,c], degrees=True)

        # jx, jy, jz = joint
        jx, jy, jz = r.apply(joint)

        return float(np.sqrt(arm2 - (tx - jx - x)**2 
                              - (ty - jy - y)**2) + jz + z)

    # Return a list of stepper positions for the given effector coordinates
    def calc_actuator(self, coordinates):
        return [self._cartesian_to_actuator(coordinates, arm2, tower, joint)
                for arm2, tower, joint in zip(self.arm2, self.towers, self.joints)]

    ######  Forward Kinematics  ######
    
    # Calculate the cartesian (X,Y,Z,A,B,C) position and orientation of the effector
    # from the positions of the stepper motors
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    # Get the Jacobian matrix for change in stepper positions as a function of effector coordinates
    # Coordinates is a list or tuple of 3 floats for X,Y,Z coordinates and A,B,C Euler angles, while
    # delta_position is half of the amount of displacement for calculating the derivatives.
    def get_jacobian(self, coordinates, delta_position = 0.001, delta_rotation = 0.001):
        coordinates = np.array(coordinates)
                
        # Declare the matrix for the result
        jacT = np.ones(shape=(6,6))

        # Numerically calculate the derivative
        for axis in range(6):
            delta = [0,0,0,0,0,0]
            if axis < 3:
                delta[axis] = delta_position
            else:
                delta[axis] = delta_rotation

            positive = np.array(self.calc_actuator(coordinates + delta))
            negative = np.array(self.calc_actuator(coordinates - delta))

            jacT[axis,:] = (positive-negative) / (2*delta[axis])

        return jacT.T

    # Calculate the cartesian coordinates for a given set of stepper positions
    def _actuator_to_cartesian(self, spos, initial_guess=None):
        max_iterations = 100
        convergence = 1e-10

        logging.info("Forward Kinematics starting, solving for stepper positions (%.3f,%.3f,%.3f,%.3f,%.3f,%.3f) mm..."
                      % (spos[0],spos[1],spos[2],spos[3],spos[4],spos[5]))

        # If no initial guess is supplied, then assume halfway up inside the build volume
        if initial_guess is None:
            current_guess = np.array([0., 0., (self.max_z-self.min_z)/2., 0., 0. ,0.])
        else:
            current_guess = np.array(initial_guess)

        for q in range(max_iterations):
            logging.info("Iteration %d: Current guess = (%.3f,%.3f,%.3f) mm, (%.3f,%.3f,%.3f) deg" % (q,
                current_guess[0],current_guess[1],current_guess[2],current_guess[3],current_guess[4],current_guess[5]))

            # Refine the cartesian coordinates guess using the Newton-Raphson method
            new_guess = current_guess - np.matmul(np.linalg.inv(self.get_jacobian(current_guess))
                                                  , np.array(self.calc_actuator(current_guess)) - spos)

            # Calculate the size of the change during this iteration
            delta = np.sum(np.abs((current_guess-new_guess)/(current_guess+new_guess)))

            # Test for successful convergence to stop the solver
            if delta <= convergence:
                logging.info("Convergence criterion achieved: %g < %g" % (delta, convergence))
                break
            
            # Setup for next iteration
            current_guess = new_guess

        logging.info("Cartesian coordinates calculated to be (%.3f,%.3f,%.3f) mm, (%.3f,%.3f,%.3f) deg" % (
                current_guess[0],current_guess[1],current_guess[2],current_guess[3],current_guess[4],current_guess[5]))

        # Ensure that the result is in native Python float types, not NumPy
        return [float(coordinate) for coordinate in current_guess]

    # def update_effector(self):
    #     # self.offset_angle += 5
    #     self._calculate_effector_joints(self.offset_angle)
    #     ffi_main, ffi_lib = chelper.get_ffi()
    #     for r, j in zip(self.rails, self.joints):
    #         for stepper in r.get_steppers():
    #             # logging.info("Updating effector: %.3f, %.3f" % (r.__str__(), s.__str__()))
    #             sk = stepper.get_stepper_kinematics()
    #             # ffi_lib.hexapod_set_params(sk, ffi_main.cast("double", j[0]), ffi_main.cast("double", j[1]), ffi_main.cast("double", j[2]))



##########################


    # Homing the printer
    def clear_homing_state(self, clear_axes):
        # Clearing homing state for each axis individually is not implemented
        if clear_axes:
            self.limit_xy2 = -1
            self.need_home = True
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        # Klipper natively supports only (X,Y,Z) coordinates
        forcepos = list(self.home_position)[:3]
        # This forces the Z value... It's from the Delta code so needs to be changed...
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        # Klipper natively supports only (X,Y,Z) coordinates
        homing_state.home_rails(self.rails, forcepos, self.home_position[:3])


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
        stepdists = [rail.get_steppers()[0].get_step_dist() for rail in self.rails]
        return HexapodCalibration(self.radius, self.angles, self.arm_lengths,
                                self.endstops, stepdists)


class HexapodRotationAxis:
    def __init__(self, kinematics, gcode_id, initial_position=0.):
        self.kinematics = kinematics
        self._gcode_id = gcode_id
        self.commanded_pos = initial_position
        logging.info("Loaded Hexapod Rotation axis %s." % (self._gcode_id))

    def calc_junction(self, prev_move, move, axis_index):
        return move.max_cruise_v2

    def find_past_position(self, print_time):
        return 0.

    def process_move(self, next_move_time, move, axis_index):
        self.commanded_pos = self.kinematics.toolhead.commanded_pos[axis_index]
        logging.info("Rotation axis %s: Process Move from (%.3f,%.3f,%.3f) to (%.3f,%.3f,%.3f) mm" %
                        (self.get_axis_gcode_id(),move.start_pos[0],move.start_pos[1],move.start_pos[2],move.end_pos[0],move.end_pos[1],move.end_pos[2]))
        logging.info("Dump start position:")
        for d in move.start_pos:
            logging.info("%.3f"%(d))
        logging.info("Dump end position:")
        for d in move.end_pos:
            logging.info("%.3f"%(d))
        logging.info("Rotation axis %s: Process Move to %.3f degrees" % (self.get_axis_gcode_id(),self.kinematics.toolhead.commanded_pos[axis_index]))

        start_pos = move.start_pos[:3]
        start_rot = scipy.spatial.transform.Rotation.from_euler('zyx', [move.start_pos[5],move.start_pos[4],move.start_pos[3]], degrees=True)

        end_pos = move.end_pos[:3]
        end_rot = scipy.spatial.transform.Rotation.from_euler('ZYX', [move.end_pos[5],move.end_pos[4],move.end_pos[3]], degrees=True)
        
        ffi_main, ffi_lib = chelper.get_ffi()
        for rail in self.kinematics.rails:
            steppers = rail.get_steppers()
            for stepper in steppers:
                sk = stepper.get_stepper_kinematics()
                ffi_lib.hexapod_stepper_set_rotation_move(sk, list(start_pos), list(start_rot.as_quat(scalar_first=True)),
                                                      list(end_pos), list(end_rot.as_quat(scalar_first=True)));

    def check_move(self, move, axis_index):
        logging.info("Rotation axis %s: Check Move from (%.3f,%.3f,%.3f) to (%.3f,%.3f,%.3f) mm" %
                        (self.get_axis_gcode_id(),move.start_pos[0],move.start_pos[1],move.start_pos[2],move.end_pos[0],move.end_pos[1],move.end_pos[2]))
        logging.info("Dump start position:")
        for d in move.start_pos:
            logging.info("%.3f"%(d))
        logging.info("Dump end position:")
        for d in move.end_pos:
            logging.info("%.3f"%(d))
        logging.info("Rotation axis %s: Check Move to %.3f degrees" % (self.get_axis_gcode_id(),self.kinematics.toolhead.commanded_pos[axis_index]))

    def get_trapq(self):
        return None

    def get_name(self):
        return ""
    
    def get_axis_gcode_id(self):
        return self._gcode_id

class HexapodMove:
    def __init__(self):
        self.active = False
        # self.start_position = 

# Delta parameter calibration for DELTA_CALIBRATE tool
class HexapodCalibration:
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
        return HexapodCalibration(radius, angles, arms, endstops, stepdists)
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
    return HexapodKinematics(toolhead, config)

