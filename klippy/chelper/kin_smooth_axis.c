// Kinematic filter to smooth out cartesian XY movements
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "integrate.h" // integrate_weighted
#include "itersolve.h" // struct stepper_kinematics
#include "scurve.h" // scurve_copy_scaled
#include "trapq.h" // trapq_integrate

// Calculate the definitive integral on part of a move
static double
move_integrate(const struct move *m, int axis, double start, double end
               , double time_offset, const struct smoother *sm
               , double damping_comp, double accel_comp)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_pos = m->start_pos.axis[axis - 'x'];
    struct scurve s;
    scurve_copy_scaled(&m->s, axis_r, &s);
    double res = 0.;
    if (accel_comp) {
        res += accel_comp * integrate_velocity_jumps(
                sm, &s, start, end, time_offset);
        start_pos += scurve_add_2nd_deriv(&m->s, axis_r * accel_comp, &s);
    }
    if (damping_comp)
        start_pos += scurve_add_deriv(&m->s, axis_r * damping_comp, &s);
    res = integrate_weighted(sm, start_pos, &s, start, end, time_offset);
    return res;
}

// Calculate the definitive integral for a range of moves
static double
range_integrate(const struct move *m, int axis, double move_time
                , const struct smoother *sm
                , double damping_comp, double accel_comp)
{
    // Calculate integral for the current move
    double start = move_time - sm->hst, end = move_time + sm->hst;
    double offset = -move_time;
    double res = move_integrate(m, axis, start, end, offset, sm
                                , damping_comp, accel_comp);
    // Integrate over previous moves
    const struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        offset -= prev->move_t;
        res += move_integrate(prev, axis, start, prev->move_t, offset, sm
                              , damping_comp, accel_comp);
    }
    // Integrate over future moves
    offset = -move_time;
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        offset += m->move_t;
        m = list_next_entry(m, node);
        res += move_integrate(m, axis, 0., end, offset, sm
                              , damping_comp, accel_comp);
    }
    return res;
}

// Calculate average position over smooth_time window
static inline double
calc_position(const struct move *m, int axis, double move_time
              , const struct smoother *sm
              , double damping_comp, double accel_comp)
{
    double area = range_integrate(m, axis, move_time, sm
                                  , damping_comp, accel_comp);
    return area * sm->inv_norm;
}

struct smooth_axis {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct smoother *x_smoother, *y_smoother;
    double x_accel_comp, y_accel_comp;
    double x_damping_comp, y_damping_comp;
    struct move m;
};

#define DUMMY_T 500.0

// Optimized calc_position when only x axis is needed
static double
smooth_x_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    if (!sa->x_smoother)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    sa->m.start_pos.x = calc_position(m, 'x', move_time, sa->x_smoother
                                      , sa->x_damping_comp, sa->x_accel_comp);
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

// Optimized calc_position when only y axis is needed
static double
smooth_y_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    if (!sa->y_smoother)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    sa->m.start_pos.y = calc_position(m, 'y', move_time, sa->y_smoother
                                      , sa->y_damping_comp, sa->y_accel_comp);
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

// General calc_position for both x and y axes
static double
smooth_xy_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    if (!sa->x_smoother && !sa->y_smoother)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    sa->m.start_pos = move_get_coord(m, move_time);
    if (sa->x_smoother)
        sa->m.start_pos.x = calc_position(m, 'x', move_time, sa->x_smoother
                                          , sa->x_damping_comp
                                          , sa->x_accel_comp);
    if (sa->y_smoother)
        sa->m.start_pos.y = calc_position(m, 'y', move_time, sa->y_smoother
                                          , sa->y_damping_comp
                                          , sa->y_accel_comp);
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

void __visible
smooth_axis_set_time(struct stepper_kinematics *sk
                     , double smooth_x, double smooth_y)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    free(sa->x_smoother);
    free(sa->y_smoother);
    double x_hst = .5 * smooth_x, y_hst = .5 * smooth_y;
    sa->x_smoother = x_hst ? alloc_smoother(x_hst) : NULL;
    sa->y_smoother = y_hst ? alloc_smoother(y_hst) : NULL;

    double hst = 0.;
    if (sa->sk.active_flags & AF_X)
        hst = x_hst;
    if (sa->sk.active_flags & AF_Y)
        hst = y_hst > hst ? y_hst : hst;
    sa->sk.gen_steps_pre_active = sa->sk.gen_steps_post_active = hst;
}

void __visible
smooth_axis_set_damping_comp(struct stepper_kinematics *sk
                             , double damping_comp_x, double damping_comp_y)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    sa->x_damping_comp = damping_comp_x;
    sa->y_damping_comp = damping_comp_y;
}

void __visible
smooth_axis_set_accel_comp(struct stepper_kinematics *sk
                           , double accel_comp_x, double accel_comp_y)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    sa->x_accel_comp = accel_comp_x;
    sa->y_accel_comp = accel_comp_y;
}

int __visible
smooth_axis_set_sk(struct stepper_kinematics *sk
                   , struct stepper_kinematics *orig_sk)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    int af = orig_sk->active_flags & (AF_X | AF_Y);
    if (af == (AF_X | AF_Y))
        sa->sk.calc_position_cb = smooth_xy_calc_position;
    else if (af & AF_X)
        sa->sk.calc_position_cb = smooth_x_calc_position;
    else if (af & AF_Y)
        sa->sk.calc_position_cb = smooth_y_calc_position;
    else
        return -1;
    sa->sk.active_flags = orig_sk->active_flags;
    sa->orig_sk = orig_sk;
    return 0;
}

struct stepper_kinematics * __visible
smooth_axis_alloc(void)
{
    struct smooth_axis *sa = malloc(sizeof(*sa));
    memset(sa, 0, sizeof(*sa));
    sa->m.move_t = 2. * DUMMY_T;
    return &sa->sk;
}
