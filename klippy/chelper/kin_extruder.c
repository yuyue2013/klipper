// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

// Calculate the definitive integral of extruder position
static double
extruder_integrate(struct move *m, double start, double end)
{
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double base = m->start_pos.x + pressure_advance * m->start_v;
    double start_v = m->start_v + pressure_advance * 2. * m->half_accel;
    // Calculate definitive integral
    double half_v = .5 * start_v, sixth_a = (1. / 3.) * m->half_accel;
    double si = start * (base + start * (half_v + start * sixth_a));
    double ei = end * (base + end * (half_v + end * sixth_a));
    return ei - si;
}

// Calculate the definitive integral of time weighted extruder position
static double
extruder_integrate_time(struct move *m, double start, double end)
{
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double base = m->start_pos.x + pressure_advance * m->start_v;
    double start_v = m->start_v + pressure_advance * 2. * m->half_accel;
    // Calculate definitive integral
    double half_b = .5 * base, third_v = (1. / 3.) * start_v;
    double eighth_a = .25 * m->half_accel;
    double si = start * start * (half_b + start * (third_v + start * eighth_a));
    double ei = end * end * (half_b + end * (third_v + end * eighth_a));
    return ei - si;
}

// Calculate the definitive integral of extruder for a given move
static double
pa_move_integrate(struct move *m, double start, double end, double time_offset)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    double iextruder = extruder_integrate(m, start, end);
    double wgt_iextruder = extruder_integrate_time(m, start, end);
    return wgt_iextruder - time_offset * iextruder;
}

// Calculate the definitive integral of the extruder over a range of moves
static double
pa_range_integrate(struct move *m, double move_time, double hst)
{
    // Calculate integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    res += pa_move_integrate(m, start, move_time, start);
    res -= pa_move_integrate(m, move_time, end, end);
    // Integrate over previous moves
    struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        res += pa_move_integrate(prev, start, prev->move_t, start);
    }
    // Integrate over future moves
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        m = list_next_entry(m, node);
        res -= pa_move_integrate(m, 0., end, end);
    }
    return res;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_half_smooth_time2;
};

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = es->half_smooth_time;
    if (!hst)
        // Pressure advance not enabled
        return m->start_pos.x + move_get_distance(m, move_time);
    // Apply pressure advance and average over smooth_time
    double area = pa_range_integrate(m, move_time, hst);
    return area * es->inv_half_smooth_time2;
}

void __visible
extruder_set_smooth_time(struct stepper_kinematics *sk, double smooth_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = smooth_time * .5;
    es->half_smooth_time = hst;
    es->sk.gen_steps_pre_active = es->sk.gen_steps_post_active = hst;
    if (! hst)
        return;
    es->inv_half_smooth_time2 = 1. / (hst * hst);
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X;
    return &es->sk;
}
