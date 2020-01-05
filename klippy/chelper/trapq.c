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
    struct trap_accel_decel *accel_decel = malloc(sizeof(*accel_decel));
    memset(accel_decel, 0, sizeof(*accel_decel));
    return accel_decel;
}

// Fill trap_accel_decel with a simple (non-combined) velocity trapezoid
void __visible
accel_decel_fill(struct trap_accel_decel *accel_decel
                 , double accel_t, double cruise_t, double decel_t
                 , double start_v, double cruise_v
                 , double accel, int accel_order)
{
    memset(accel_decel, 0, sizeof(*accel_decel));
    accel_decel->accel_order = accel_order;
    accel_decel->accel_t = accel_decel->uncomp_accel_t = accel_t;
    accel_decel->total_accel_t = accel_t;
    accel_decel->cruise_t = cruise_t;
    accel_decel->decel_t = accel_decel->uncomp_decel_t = decel_t;
    accel_decel->total_decel_t = decel_t;
    accel_decel->start_accel_v = start_v;
    accel_decel->cruise_v = cruise_v;
    accel_decel->effective_accel = accel_decel->effective_decel = accel;
}

// Fill and add a move to the trapezoid velocity queue
void __visible
trapq_append(struct trapq *tq, double print_time
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_r_x, double axes_r_y, double axes_r_z
             , const struct trap_accel_decel *accel_decel)
{
    struct coord start_pos = { .x=start_pos_x, .y=start_pos_y, .z=start_pos_z };
    struct coord axes_r = { .x=axes_r_x, .y=axes_r_y, .z=axes_r_z };
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

#define NEVER_TIME 9999999999999999.9

// Allocate a new 'trapq' object
struct trapq * __visible
trapq_alloc(void)
{
    struct trapq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    struct move *head_sentinel = move_alloc(), *tail_sentinel = move_alloc();
    tail_sentinel->print_time = tail_sentinel->move_t = NEVER_TIME;
    list_add_head(&head_sentinel->node, &tq->moves);
    list_add_tail(&tail_sentinel->node, &tq->moves);
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

// Update the list sentinels
void
trapq_check_sentinels(struct trapq *tq)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    if (tail_sentinel->print_time)
        // Already up to date
        return;
    struct move *m = list_prev_entry(tail_sentinel, node);
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    if (m == head_sentinel) {
        // No moves at all on this list
        tail_sentinel->print_time = NEVER_TIME;
        return;
    }
    tail_sentinel->print_time = m->print_time + m->move_t;
    tail_sentinel->start_pos = move_get_coord(m, m->move_t);
}

#define MAX_NULL_MOVE 1.0

// Add a move to the trapezoid velocity queue
void
trapq_add_move(struct trapq *tq, struct move *m)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    struct move *prev = list_prev_entry(tail_sentinel, node);
    if (prev->print_time + prev->move_t < m->print_time) {
        // Add a null move to fill time gap
        struct move *null_move = move_alloc();
        null_move->start_pos = m->start_pos;
        if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
            // Limit the first null move to improve numerical stability
            null_move->print_time = m->print_time - MAX_NULL_MOVE;
        else
            null_move->print_time = prev->print_time + prev->move_t;
        null_move->move_t = m->print_time - null_move->print_time;
        list_add_before(&null_move->node, &tail_sentinel->node);
    }
    list_add_before(&m->node, &tail_sentinel->node);
    tail_sentinel->print_time = 0.;
}

// Free any moves older than `print_time` from the trapezoid velocity queue
void __visible
trapq_free_moves(struct trapq *tq, double print_time)
{
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    for (;;) {
        struct move *m = list_next_entry(head_sentinel, node);
        if (m == tail_sentinel) {
            tail_sentinel->print_time = NEVER_TIME;
            return;
        }
        if (m->print_time + m->move_t > print_time)
            return;
        list_del(&m->node);
        free(m);
    }
}
