// Kinematic filter to smooth out cartesian XY movements
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // trapq_integrate

struct smooth_axis {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    double x_half_smooth_time, x_inv_smooth_time;
    double y_half_smooth_time, y_inv_smooth_time;
    struct move m;
};

#define DUMMY_T 500.0

static double
smooth_x_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    double hst = sa->x_half_smooth_time;
    if (!hst)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    // Calculate average position over smooth_time window
    double area = trapq_integrate(m, 'x', move_time - hst, move_time + hst);
    sa->m.start_pos.x = area * sa->x_inv_smooth_time;
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

static double
smooth_y_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    double hst = sa->y_half_smooth_time;
    if (!hst)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    // Calculate average position over smooth_time window
    double area = trapq_integrate(m, 'y', move_time - hst, move_time + hst);
    sa->m.start_pos.y = area * sa->y_inv_smooth_time;
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

static double
smooth_xy_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    double x_hst = sa->x_half_smooth_time;
    double y_hst = sa->y_half_smooth_time;
    if (!x_hst && !y_hst)
        return sa->orig_sk->calc_position_cb(sa->orig_sk, m, move_time);
    sa->m.start_pos = move_get_coord(m, move_time);
    if (x_hst) {
        double area = trapq_integrate(m, 'x', move_time-x_hst, move_time+x_hst);
        sa->m.start_pos.x = area * sa->x_inv_smooth_time;
    }
    if (y_hst) {
        double area = trapq_integrate(m, 'y', move_time-y_hst, move_time+y_hst);
        sa->m.start_pos.y = area * sa->y_inv_smooth_time;
    }
    return sa->orig_sk->calc_position_cb(sa->orig_sk, &sa->m, DUMMY_T);
}

void __visible
smooth_axis_set_time(struct stepper_kinematics *sk
                     , double smooth_x, double smooth_y)
{
    struct smooth_axis *sa = container_of(sk, struct smooth_axis, sk);
    sa->x_half_smooth_time = .5 * smooth_x;
    sa->x_inv_smooth_time = 1. / smooth_x;
    sa->y_half_smooth_time = .5 * smooth_y;
    sa->y_inv_smooth_time = 1. / smooth_y;

    double hst = 0.;
    if (sa->sk.active_flags & AF_X)
        hst = sa->x_half_smooth_time;
    if (sa->sk.active_flags & AF_Y)
        hst = sa->y_half_smooth_time > hst ? sa->y_half_smooth_time : hst;
    sa->sk.scan_past = sa->sk.scan_future = hst;
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
