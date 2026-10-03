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


// // Store an angle, with pre-calculated sine and cosine (versine)
// struct angle {
//     double angle;       // Stores the angle
//     double sin;         // sin(angle)
//     double cos;         // cos(angle)
// };

struct hexapod_stepper {
    struct stepper_kinematics sk;

    /* Fixed configuration */
    double arm2;            // arm length squared
    struct coord tower;     // coordinates of axis tower joint (X and Y only, Z is reserved)
    struct coord joint;     // coordinates of the joint on the effector (relative to the effector datum)

    /* Effector state/offset */
    // Quaternion orientation;
    // struct coord offset;

    /* How to rotate during XYZ move: (start_pos,start_rot) --> (end_pos,end_rot) */
    int enable_rot;
    struct coord start_pos, end_pos; // Start and end of overall move coordinate
    double dist2_rot;               // Squared distance to overall move start point
    Quaternion start_rot, end_rot;  // Initial rotation and Final rotation
};


void hexapod_stepper_calc_joint(struct stepper_kinematics *sk, struct coord *out, double t)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    Quaternion step_rotation;
        
    // Proportional of distance travelled at the current point in the move
    // double t = move_get_distance(m, move_time) / move_get_distance(m, m->move_t);

    // Use spherical linear interpolation to get the correct rotation
    Quaternion_slerp(&ds->start_rot, &ds->end_rot, t, &step_rotation);

    // Rotate the joint into position
    Quaternion_rotate(&step_rotation, ds->joint.axis, out->axis);
}

static double hexapod_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    struct coord effector_position = move_get_coord(m, move_time);
    struct coord working;   // Working position of the effector joint
    struct coord d;
    double t;

    // Skip rotation calculations if the rotation rate is zero
    if(ds->enable_rot == 1) {
        d.x = ds->end_pos.x - effector_position.x;
        d.y = ds->end_pos.y - effector_position.y;
        d.z = ds->end_pos.z - effector_position.z;
        
        t = ds->dist2_rot - d.x*d.x - d.y*d.y - d.z*d.z;
        
        // Rotate in proportion to how close to the end position (safe if movement continues)
        if (t <= 0.0) t = 0.0;
        hexapod_stepper_calc_joint(sk, &working, t / ds->dist2_rot);

    } else {
        working.x = ds->joint.x;
        working.y = ds->joint.y;
        working.z = ds->joint.z;
    }

    // Calculate X,Y,Z offset between effector joint and printer rail
    double dx = ds->tower.x - (effector_position.x + working.x);
    double dy = ds->tower.y - (effector_position.y + working.y);
    double dz = effector_position.z + working.z;

    // Calculate the correct position for the printer rail at this point in the move
    return sqrt(ds->arm2 - dx*dx - dy*dy) + dz;
}

struct stepper_kinematics * __visible
hexapod_stepper_alloc(double arm2, double tower_x, double tower_y, double tower_z, double joint_x, double joint_y, double joint_z)
{
    struct hexapod_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));

    struct coord d;

    ds->arm2 = arm2;
    ds->tower.x = tower_x;
    ds->tower.y = tower_y;
    ds->tower.z = tower_z;
    ds->joint.x = joint_x;
    ds->joint.y = joint_y;
    ds->joint.z = joint_z;

    ds->enable_rot = true;
    ds->start_pos.x = 20.0f;
    ds->start_pos.y = 10.0f;
    ds->start_pos.z = 50.0f;
    ds->end_pos.x = 50.0f;
    ds->end_pos.y = 10.0f;
    ds->end_pos.z = 50.0f;

    d.x = ds->end_pos.x - ds->start_pos.x;
    d.y = ds->end_pos.y - ds->start_pos.y;
    d.z = ds->end_pos.z - ds->start_pos.z;
    ds->dist2_rot = d.x*d.x + d.y*d.y + d.z*d.z;
    
    Quaternion_fromZRotation(0.0 / 180.0 * 3.141592, &ds->start_rot);
    Quaternion_fromZRotation(15.0 / 180.0 * 3.141592, &ds->end_rot);

    ds->sk.calc_position_cb = hexapod_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
