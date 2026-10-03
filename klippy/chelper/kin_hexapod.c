// Sliding parallel kinematics (6 Degrees of Freedom) stepper pulse time generation
//
// Copyright (C) 2026 Peter Spencer
// Code dervied from kin_delta.c by Kevin O'Connor
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stdbool.h>
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset

#include "Quaternion.h"

#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

// Coordinates to describe the effector orientation at an (X,Y,Z) position
struct hexapod_coord {
    struct coord position;      // Position in mm
    Quaternion orientation;     // Rotation that orients the effector
};

struct hexapod_move {
    struct hexapod_coord start; // Start of movement
    struct hexapod_coord end;   // End of movement
    double distance;            // Linear distance between start and end, cached
    bool active;                // Rotational movement, enable the extra calculations
};

struct hexapod_stepper {
    struct stepper_kinematics sk;

    /* Fixed configuration */
    double arm2;                // arm length squared (mm^2)
    struct coord tower;         // coordinates of axis tower joint (X and Y only, Z is reserved) (mm)
    struct coord joint;         // coordinates of the joint on the effector (relative to the effector datum) (mm)

    /* State information */
    struct hexapod_move move;   // The movement command for rotary motion
    Quaternion orientation;     // the rotation for current orientation of the effector
    struct coord joint_actual;  // coordinates of effector joint under rotation (mm)
};

/*********** Private helper functions ***********/

// Setter for the hexapod_coord data structure
static void _hexapod_stepper_set_coordinate(double position[3], double orientation[4], struct hexapod_coord *output)
{
    output->position.x = position[0];
    output->position.y = position[1];
    output->position.z = position[2];
    Quaternion_set(orientation[0],orientation[1],orientation[2],orientation[3], &output->orientation);
}

// Return the linear distance between two points
static double _hexapod_stepper_get_distance(struct coord *start_position, struct coord *end_position)
{
    double dx = end_position->x - start_position->x;
    double dy = end_position->y - start_position->y;
    double dz = end_position->z - start_position->z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Rotate the effector and set whether to continue calculations
static void _hexapod_stepper_set_rotation(struct stepper_kinematics *sk, Quaternion *rotation, bool enable)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);

    // Disable movement
    ds->move.active = false;
    
    // Set the current rotation transformation for orienting the effector
    if (rotation != &ds->orientation) {
        Quaternion_copy(rotation, &ds->orientation);
    }

    // Rotate the joint around the effector datum/origin point
    Quaternion_rotate(&ds->orientation, ds->joint.axis, ds->joint_actual.axis);

    // Set movement state going forward
    ds->move.active = enable;
}

/*********** Public functions for controlling the rotational movement ***********/

// Stop the extra calculations
void __visible hexapod_stepper_lock_rotation(struct stepper_kinematics *sk)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    ds->move.active = false;
}

// Reset the effector position and stop the extra calculations
// DANGER: May cause sudden movements and invalidate kinematics
void __visible hexapod_stepper_clear_rotation(struct stepper_kinematics *sk)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    Quaternion_setIdentity(&ds->orientation);
    _hexapod_stepper_set_rotation(sk, &ds->orientation, false);
}

// Get the current move command and active status
struct __visible hexapod_move *hexapod_stepper_get_rotation_move(struct stepper_kinematics *sk)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    return &ds->move;
}

// Setup the rotation movement and activate extra calculations
void __visible hexapod_stepper_set_rotation_move(struct stepper_kinematics *sk, double start_pos[3], double start_rot[4], double end_pos[3], double end_rot[4])
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);

    // Disable rotation re-calculation
    ds->move.active = false;

    // Set the start and end coordinates
    _hexapod_stepper_set_coordinate(start_pos, start_rot, &ds->move.start);
    _hexapod_stepper_set_coordinate(end_pos, end_rot, &ds->move.end);

    // Calculate the linear distance between the start and end points
    ds->move.distance = _hexapod_stepper_get_distance(&ds->move.start.position, &ds->move.end.position);
    
    // Enable rotation re-calculation
    ds->move.active = true;
}

// Complete the rotation movement by finalising the orientation and stopping extra calculations
void __visible hexapod_stepper_finish_rotation_move(struct stepper_kinematics *sk)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);

    // Set the orientation state to be the final orientation
    // transformation and stop the move
    _hexapod_stepper_set_rotation(sk, &ds->move.end.orientation, false);
}

/*********** Public functions for TESTING ONLY ***********/

// Hard-coded rotation move for testing only
void __visible hexapod_stepper_test_rotation(struct stepper_kinematics *sk)
{
    Quaternion start_orientation, end_orientation;
    double start_rot[4], end_rot[4];

    // Set the start and end coordinates
    double start_position[3] = { 0.0, 0.0, 55.0 };
    double start_angle = 0.0;

    double end_position[3] = { 0.0, 0.0, 50.0 };
    double end_angle = 0.0;

    Quaternion_fromZRotation(start_angle / 180.0 * 3.141592, &start_orientation);
    start_rot[0] = start_orientation.w;
    start_rot[1] = start_orientation.v[0];
    start_rot[2] = start_orientation.v[1];
    start_rot[3] = start_orientation.v[2];

    Quaternion_fromZRotation(end_angle / 180.0 * 3.141592, &end_orientation);
    end_rot[0] = end_orientation.w;
    end_rot[1] = end_orientation.v[0];
    end_rot[2] = end_orientation.v[1];
    end_rot[3] = end_orientation.v[2];

    hexapod_stepper_set_rotation_move(sk, start_position, start_rot, end_position, end_rot);
}

/*********** The main kinematics functions for Klipper ***********/

static double hexapod_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    struct coord effector_position = move_get_coord(m, move_time);
    double t;

    // Execture an active rotation movement
    if(ds->move.active) {
        // Calculate the "closeness" to the end position:
        // 0 = further away that the start position, 1 = located at the end position
        // This is safe because if movement continues before rotation command is updated there
        // will not be any sudden discontinuous motion.
        t = 1.0 - _hexapod_stepper_get_distance(&effector_position, &ds->move.end.position) / ds->move.distance;
        
        if (t > 0.0) {
            // Rotate in proportion to how close the effector is to the end position
            // Use spherical linear interpolation to get the correct rotation
            Quaternion_slerp(&ds->move.start.orientation, &ds->move.end.orientation, t, &ds->orientation);

            // Rotate the effector
            _hexapod_stepper_set_rotation(sk, &ds->orientation, true);
        }
    }

    // Calculate X,Y,Z offset between effector joint and printer rail
    double dx = ds->tower.x - (effector_position.x + ds->joint_actual.x);
    double dy = ds->tower.y - (effector_position.y + ds->joint_actual.y);
    double dz = effector_position.z + ds->joint_actual.z;

    // Calculate the correct position for the printer rail at this point in the move
    return sqrt(ds->arm2 - dx*dx - dy*dy) + dz;
}


struct stepper_kinematics * __visible
hexapod_stepper_alloc(double arm2, struct coord *tower, struct coord *joint)
//hexapod_stepper_alloc(double arm2, double tower_x, double tower_y, double tower_z, double joint_x, double joint_y, double joint_z)
{
    struct hexapod_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));

    ds->arm2 = arm2;
    
    ds->tower.x = tower->x;
    ds->tower.y = tower->y;
    ds->tower.z = tower->z;
    ds->joint.x = joint->x;
    ds->joint.y = joint->y;
    ds->joint.z = joint->z;
    // ds->tower.x = tower_x;
    // ds->tower.y = tower_y;
    // ds->tower.z = tower_z;
    // ds->joint.x = joint_x;
    // ds->joint.y = joint_y;
    // ds->joint.z = joint_z;
    
    // Setup state information (no rotation)
    hexapod_stepper_clear_rotation(&ds->sk);

    // Hard coded rotation move for testing only
    hexapod_stepper_test_rotation(&ds->sk);

    ds->sk.calc_position_cb = hexapod_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
