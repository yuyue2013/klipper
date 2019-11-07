// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance
#include "scurve.h" // scurve_fill

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    return m->start_pos.x + move_get_distance(m, move_time);
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    sk->calc_position_cb = extruder_calc_position;
    sk->active_flags = AF_X;
    return sk;
}

// Populate a 'struct move' with an extruder velocity trapezoid
void __visible
extruder_add_move(struct trapq *tq, double print_time
                  , double start_e_pos, double extrude_r
                  , const struct trap_accel_decel *accel_decel)
{
    struct coord start_pos, axes_r;
    start_pos.x = start_e_pos;
    axes_r.x = 1.;
    start_pos.y = start_pos.z = axes_r.y = axes_r.z = 0.;

    // NB: acceleration compensation reduces duration of moves in the beginning
    // of acceleration move group, and increases it in case of deceleration.
    // The extruder kinematics does not follow acceleration compensation, so
    // print_time must be adjusted accordingly to track the start and the
    // duration of the non-compensated moves.
    if (accel_decel->total_accel_t)
        print_time += accel_decel->uncomp_accel_offset_t
            - accel_decel->accel_offset_t;
    else if (accel_decel->total_decel_t)
        print_time += accel_decel->uncomp_decel_offset_t
            - accel_decel->decel_offset_t;

    if (accel_decel->uncomp_accel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->uncomp_accel_t;
        scurve_fill(&m->s, accel_decel->accel_order,
                accel_decel->uncomp_accel_t, accel_decel->uncomp_accel_offset_t,
                accel_decel->total_accel_t,
                extrude_r * accel_decel->start_accel_v,
                extrude_r * accel_decel->effective_accel, 0.);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += accel_decel->uncomp_accel_t;
        start_pos.x += move_get_distance(m, accel_decel->uncomp_accel_t);
    }
    if (accel_decel->cruise_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->cruise_t;
        scurve_fill(&m->s, 2,
                accel_decel->cruise_t, 0., accel_decel->cruise_t,
                extrude_r * accel_decel->cruise_v, 0., 0.);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += accel_decel->cruise_t;
        start_pos.x += move_get_distance(m, accel_decel->cruise_t);
    }
    if (accel_decel->uncomp_decel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->uncomp_decel_t;
        scurve_fill(&m->s, accel_decel->accel_order, accel_decel->uncomp_decel_t,
                accel_decel->uncomp_decel_offset_t, accel_decel->total_decel_t,
                extrude_r * accel_decel->cruise_v,
                -extrude_r * accel_decel->effective_decel, 0.);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);
    }
}
