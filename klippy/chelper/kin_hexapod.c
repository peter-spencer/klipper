// Sliding parallel kinematics (6 Degrees of Freedom) stepper pulse time generation
//
// Copyright (C) 2026 Peter Spencer
// Code dervied from kin_delta.c by Kevin O'Connor
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct hexapod_stepper {
    struct stepper_kinematics sk;
    double arm2, tower_x, tower_y, joint_x, joint_y, joint_z;
};

static double
hexapod_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct hexapod_stepper *ds = container_of(sk, struct hexapod_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    double dx = ds->tower_x - (c.x + ds->joint_x);
    double dy = ds->tower_y - (c.y + ds->joint_y);
    double dz = c.z + ds->joint_z;

    return sqrt(ds->arm2 - dx*dx - dy*dy) + dz;
}

struct stepper_kinematics * __visible
hexapod_stepper_alloc(double arm2, double tower_x, double tower_y, double joint_x, double joint_y, double joint_z)
{
    struct sliding_parallel_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->arm2 = arm2;
    ds->tower_x = tower_x;
    ds->tower_y = tower_y;
    ds->joint_x = joint_x;
    ds->joint_y = joint_y;
    ds->joint_z = joint_z;
    ds->sk.calc_position_cb = hexapod_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
