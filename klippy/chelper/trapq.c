// Trapezoidal velocity movement queue
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // unlikely
#include "trapq.h" // move_get_coord

// Allocate a new 'move' object
struct move *
move_alloc(void)
{
    struct move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// Allocate a new 'trap_accel_decel' object
struct trap_accel_decel * __visible
accel_decel_alloc(void)
{
    struct trap_accel_decel *tad = malloc(sizeof(*tad));
    memset(tad, 0, sizeof(*tad));
    return tad;
}

// Fill and add a move to the trapezoid velocity queue
void __visible
trapq_append(struct trapq *tq, double print_time
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_d_x, double axes_d_y, double axes_d_z
             , const struct trap_accel_decel *accel_decel)
{
    struct coord axes_r, start_pos;
    double inv_move_d = 1. / sqrt(axes_d_x*axes_d_x + axes_d_y*axes_d_y
                                  + axes_d_z*axes_d_z);
    axes_r.x = axes_d_x * inv_move_d;
    axes_r.y = axes_d_y * inv_move_d;
    axes_r.z = axes_d_z * inv_move_d;
    start_pos.x = start_pos_x;
    start_pos.y = start_pos_y;
    start_pos.z = start_pos_z;

    if (accel_decel->accel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->accel_t;
        scurve_fill(&m->s, accel_decel->accel_order, accel_decel->accel_t,
                accel_decel->accel_offset_t, accel_decel->total_accel_t,
                accel_decel->start_accel_v, accel_decel->effective_accel,
                accel_decel->accel_comp);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += accel_decel->accel_t;
        start_pos = move_get_coord(m, accel_decel->accel_t);
    }
    if (accel_decel->cruise_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->cruise_t;
        scurve_fill(&m->s, 2,
                accel_decel->cruise_t, 0., accel_decel->cruise_t,
                accel_decel->cruise_v, 0., 0.);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += accel_decel->cruise_t;
        start_pos = move_get_coord(m, accel_decel->cruise_t);
    }
    if (accel_decel->decel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_decel->decel_t;
        scurve_fill(&m->s, accel_decel->accel_order, accel_decel->decel_t,
                accel_decel->decel_offset_t, accel_decel->total_decel_t,
                accel_decel->cruise_v, -accel_decel->effective_decel,
                accel_decel->accel_comp);
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);
    }
}

// Return the distance moved given a time in a move
inline double
move_get_distance(struct move *m, double move_time)
{
    return scurve_eval(&m->s, move_time);
}

// Return the XYZ coordinates given a time in a move
inline struct coord
move_get_coord(struct move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    return (struct coord) {
        .x = m->start_pos.x + m->axes_r.x * move_dist,
        .y = m->start_pos.y + m->axes_r.y * move_dist,
        .z = m->start_pos.z + m->axes_r.z * move_dist };
}

// Allocate a new 'trapq' object
struct trapq * __visible
trapq_alloc(void)
{
    struct trapq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    return tq;
}

// Free memory associated with a 'trapq' object
void __visible
trapq_free(struct trapq *tq)
{
    while (!list_empty(&tq->moves)) {
        struct move *m = list_first_entry(&tq->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
    free(tq);
}

// Add a move to the trapezoid velocity queue
void
trapq_add_move(struct trapq *tq, struct move *m)
{
    list_add_tail(&m->node, &tq->moves);
}

// Free any moves older than `print_time` from the trapezoid velocity queue
void __visible
trapq_free_moves(struct trapq *tq, double print_time)
{
    while (!list_empty(&tq->moves)) {
        struct move *m = list_first_entry(&tq->moves, struct move, node);
        if (m->print_time + m->move_t > print_time)
            return;
        list_del(&m->node);
        free(m);
    }
}
