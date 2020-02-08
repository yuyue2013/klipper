// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "integrate.h" // integrate_weighted
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * w(x-t, smooth_time/2) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / definitive_integral(w(x, smooth_time/2)*dx,
//                               from=-smooth_time/2, to=smooth_time/2)

// Calculate the definitive integral of extruder for a given move
static double
pa_move_integrate(struct move *m, double start, double end, double time_offset
                  , double hst)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double base = m->start_pos.x + pressure_advance * m->start_v;
    double start_v = m->start_v + pressure_advance * 2. * m->half_accel;
    // Calculate definitive integral
    double ha = m->half_accel;
    return integrate_weighted(base, start_v, ha, start, end, time_offset, hst);
}

// Calculate the definitive integral of the extruder over a range of moves
static double
pa_range_integrate(struct move *m, double move_time, double hst)
{
    // Calculate integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    double offset = -move_time;
    res += pa_move_integrate(m, start, end, offset, hst);
    // Integrate over previous moves
    struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        offset -= prev->move_t;
        res += pa_move_integrate(prev, start, prev->move_t, offset, hst);
    }
    // Integrate over future moves
    offset = -move_time;
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        offset += m->move_t;
        m = list_next_entry(m, node);
        res += pa_move_integrate(m, 0., end, offset, hst);
    }
    return res;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_norm;
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
    return area * es->inv_norm;
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
    es->inv_norm = calc_inv_norm(hst);
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
