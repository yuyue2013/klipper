// Look-ahead movement planning queue
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <assert.h> // assert
#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "accelgroup.h"
#include "compiler.h" // __visible
#include "itersolve.h"
#include "moveq.h"
#include "pyhelper.h" // get_monotonic
#include "scurve.h" // scurve
#include "trapq.h" // trap_accel_decel
#include "trapbuild.h"

static const double EPSILON = 0.000000001;

static struct qmove *
qmove_alloc(void)
{
    struct qmove *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

struct moveq * __visible
moveq_alloc(void)
{
    struct moveq *mq = malloc(sizeof(*mq));
    memset(mq, 0, sizeof(*mq));
    list_init(&mq->moves);
    init_combiner(&mq->accel_combiner);
    return mq;
}

void __visible
moveq_reset(struct moveq *mq)
{
    struct qmove *m = NULL, *nm = NULL;
    list_for_each_entry_safe(m, nm, &mq->moves, node) {
        list_del(&m->node);
        free(m);
    }
    reset_combiner(&mq->accel_combiner);
    memset(mq, 0, sizeof(*mq));
    list_init(&mq->moves);
    init_combiner(&mq->accel_combiner);
}

static struct qmove *
backward_smoothed_pass(struct moveq *mq, int lazy, int *ret)
{
    *ret = 0;
    int update_flush_limit = lazy;
    // Traverse queue from last to first move and determine maximum
    // junction speed assuming the robot comes to a complete stop
    // after the last move.
    struct list_head delayed;
    list_init(&delayed);
    double next_smoothed_v2 = 0., peak_cruise_v2 = 0.;
    reset_junctions(&mq->accel_combiner, 0.);
    struct qmove *move = NULL, *pm = NULL, *flush_limit = NULL;
    list_for_each_entry_reversed_safe(move, pm, &mq->moves, node) {
        // Determine peak cruise velocity
        double reachable_smoothed_v2 = next_smoothed_v2 + move->smooth_delta_v2;
        double smoothed_v2 = MIN(move->max_smoothed_v2, reachable_smoothed_v2);
        if (smoothed_v2 < reachable_smoothed_v2) {
            // It's possible for this move to accelerate
            if (smoothed_v2 + move->smooth_delta_v2 > next_smoothed_v2
                    || !list_empty(&delayed)) {
                // This move can decelerate or this is a full accel
                // move after a full decel move
                if (update_flush_limit && peak_cruise_v2) {
                    flush_limit = move;
                    update_flush_limit = 0;
                }
                peak_cruise_v2 = (smoothed_v2 + reachable_smoothed_v2) * .5;
                peak_cruise_v2 = MIN(move->max_cruise_v2, peak_cruise_v2);
            }
            struct qmove *m = NULL;
            if (!update_flush_limit && move != flush_limit) {
                move->max_cruise_v2 = MIN(move->max_cruise_v2
                        , peak_cruise_v2);
                move->junction_max_v2 = MIN(move->junction_max_v2
                        , peak_cruise_v2);
                list_for_each_entry(m, &delayed, node) {
                    m->max_cruise_v2 = MIN(m->max_cruise_v2, peak_cruise_v2);
                    m->junction_max_v2 = MIN(m->junction_max_v2
                            , peak_cruise_v2);
                }
                m = list_next_entry(move, node);
                if (lazy && list_at_end(m, &mq->moves, node)) {
                    errorf("Logic error: smoothed peak velocity trapezoid at "
                            "the end of the move queue");
                    *ret = ERROR_RET;
                    return NULL;
                }
                if (!list_at_end(m, &mq->moves, node)) {
                    m->junction_max_v2 = MIN(m->junction_max_v2
                            , peak_cruise_v2);
                }
            }
            struct qmove *nm = NULL, *qm = move;
            // Put delayed moves back to their places in mq->moves list.
            list_for_each_entry_safe(m, nm, &delayed, node) {
                list_del(&m->node);
                list_add_after(&m->node, &qm->node);
                qm = m;
            }
        } else {
            // Delay calculating this move until peak_cruise_v2 is known
            list_del(&move->node);
            list_add_head(&move->node, &delayed);
        }
        if (mq->smoothed_pass_limit == move)
            break;
        next_smoothed_v2 = smoothed_v2;
    }
    if (!list_empty(&delayed)) {
        errorf("Non-empty 'delayed' queue after the smoothed pass");
        *ret = ERROR_RET;
        return NULL;
    }
    mq->smoothed_pass_limit = flush_limit;
    if (update_flush_limit) return NULL;
    return flush_limit;
}

static void
backward_pass(struct moveq *mq)
{
    // Backward pass for full acceleration
    double junction_max_v2 = 0.;
    struct qmove *move = NULL;
    list_for_each_entry_reversed(move, &mq->moves, node) {
        // Restore the default accel and decel values if they were modified
        // on previous backward pass
        move->decel_group = move->accel_group = move->default_accel;

        struct accel_group *decel = &move->decel_group;
        process_next_accel(&mq->accel_combiner, decel, junction_max_v2);
        junction_max_v2 = move->junction_max_v2;
    }
}

static struct qmove *
compute_safe_flush_limit(struct moveq *mq, int lazy, struct qmove *flush_limit)
{
    if (!lazy) return flush_limit;
    // Go over all moves from the beginning of the queue up to the current
    // flush_limit and check their deceleration paths. Make sure that all
    // flushed moves will have a sufficiently distant junction point on their
    // deceleration path with junction_max_v2 reached, and keep references to
    // such points in safe_decel in case they are needed in a forward pass.
    for (struct qmove *move = flush_limit;
            !list_at_end(move, &mq->moves, node);
            move = list_prev_entry(move, node)) {
        struct accel_group safe_decel = move->decel_group;
        safe_decel.combined_d = 0.;
        struct qmove *m, *nm = NULL;
        for (m = move; !list_at_end(m, &mq->moves, node); m = nm) {
            safe_decel.combined_d += m->decel_group.combined_d;
            limit_accel(&safe_decel, m->decel_group.max_accel
                    , m->decel_group.max_jerk);
            double min_safe_dist = calc_min_safe_dist(&safe_decel
                    , safe_decel.max_end_v2);
            struct accel_group *start_decel = m->decel_group.start_accel;
            nm = list_next_entry(start_decel->move, node);
            if (safe_decel.combined_d > min_safe_dist + EPSILON
                    && !list_at_end(nm, &mq->moves, node)
                    && nm->junction_max_v2 <= start_decel->max_start_v2) {
                move->safe_decel = safe_decel;
                move->safe_decel.move = move;
                move->safe_decel.start_accel = start_decel;
                break;
            }
        }
        if (list_at_end(m, &mq->moves, node)) {
            // The current 'move' does not have a junction point on its
            // deceleration path where junction_max_v2 is reached farther
            // than the minimum safe distance. This means that this move can
            // change its max_end_v2 if more moves are added to the queue
            // later, so it is not safe to flush moves until this move yet.
            flush_limit = move;
        }
    }
    return flush_limit;
}

static struct qmove *
forward_pass(struct moveq *mq, struct qmove *end, int lazy)
{
    struct qmove *move = list_first_entry(&mq->moves, struct qmove, node);
    double start_v2 = mq->prev_end_v2;
    double max_end_v2 = move->decel_group.max_end_v2;
    if (max_end_v2 + EPSILON < start_v2) {
        errorf("Logic error: impossible to reach the committed v2 = %.3f"
                ", max velocity = %.3lf, fallback to suboptimal planning"
                , start_v2, max_end_v2);
        struct accel_group *decel = &move->decel_group;
        double decel_start_v2 = move->safe_decel.start_accel->max_start_v2;
        // Current max_start_v2 for safe_decel->move can only be less than
        // previously captured safe_decel->max_start_v2.
        *decel = move->safe_decel;
        // Note that there is no need to check if deceleration will exceed
        // any junction_max_v2 on the way - if it did, we would have reached
        // mq->prev_end_v2 due to the choice of safe_decel->start_accel.
        decel->max_end_v2 = start_v2;
        set_max_start_v2(decel->start_accel, MIN(start_v2, decel_start_v2));
    }
    struct vtrap vt;
    vtrap_init(&vt);
    reset_junctions(&mq->accel_combiner, start_v2);
    double prev_cruise_v2 = start_v2;
    struct qmove *last_flushed_move = NULL, *next_move = NULL;
    for (; !list_at_end(move, &mq->moves, node) && move != end;
            move = next_move) {
        // Track next_move early because move will be moved to trapezoid list
        next_move = list_next_entry(move, node);
        struct accel_group *accel = &move->accel_group;
        struct accel_group *decel = &move->decel_group;

        process_next_accel(&mq->accel_combiner, accel
                , MIN(move->junction_max_v2, prev_cruise_v2));
        int can_accelerate = decel->max_end_v2 > accel->max_start_v2 + EPSILON;
        if (can_accelerate) {
            // This move can accelerate
            if (vt.decel_head) {
                // Complete the previously combined velocity trapezoid
                last_flushed_move = vtrap_flush(&vt, &move->node);
            }
            vtrap_add_as_accel(&vt, move);
        }
        int must_decelerate = accel->max_end_v2 + EPSILON > decel->max_start_v2;
        if (must_decelerate || !can_accelerate) {
            // This move must decelerate after acceleration,
            // or this is a full decel move after full accel move.
            for (; move != end; move = next_move
                    , next_move = list_next_entry(move, node)) {
                vtrap_add_as_decel(&vt, move);
                if (move == decel->start_accel->move) break;
            }
            if (move == end) break;
            reset_junctions(&mq->accel_combiner, decel->start_accel->max_start_v2);
        }
        prev_cruise_v2 = move->max_cruise_v2;
    }
    if (!lazy) {
        if (vt.decel_head)
            last_flushed_move = vtrap_flush(&vt, &mq->moves.root);
    } else {
        assert(end != NULL);
        // Just put the remaining moves back to mq->moves
        vtrap_clear(&vt, &end->node);
    }
    return last_flushed_move;
}

int __visible
moveq_add(struct moveq *mq, double move_d
          , double junction_max_v2, double velocity
          , int accel_order, double accel, double smoothed_accel
          , double jerk, double min_jerk_limit_time, double accel_comp)
{
    struct qmove *m = qmove_alloc();

    m->accel_comp = accel_comp;
    m->move_d = move_d;

    fill_accel_group(&m->default_accel, m, accel_order, accel, jerk
            , min_jerk_limit_time);
    m->max_cruise_v2 = velocity * velocity;
    m->junction_max_v2 = junction_max_v2;
    m->smooth_delta_v2 = 2. * smoothed_accel * move_d;

    if (!list_empty(&mq->moves)) {
        struct qmove *prev_move = list_last_entry(&mq->moves, struct qmove, node);
        m->max_smoothed_v2 =
            prev_move->max_smoothed_v2 + prev_move->smooth_delta_v2;
        m->max_smoothed_v2 = MIN(
                MIN(m->max_smoothed_v2, junction_max_v2),
                MIN(m->max_cruise_v2, prev_move->max_cruise_v2));
    }
    list_add_tail(&m->node, &mq->moves);
    return 0;
}

double __visible
moveq_getmove(struct moveq *mq, struct trap_accel_decel *accel_decel)
{
    memset(accel_decel, 0, sizeof(*accel_decel));
    if (list_empty(&mq->moves)) {
        errorf("Move queue is empty");
        return ERROR_RET;
    }
    struct qmove *move = list_first_entry(&mq->moves, struct qmove, node);
    accel_decel->accel_comp = move->accel_comp;
    struct accel_group *accel = &move->accel_group;
    struct accel_group *decel = &move->decel_group;
    accel_decel->accel_order = accel->accel_order;
    // Determine move velocities
    accel_decel->start_accel_v = accel->start_accel_v;
    accel_decel->cruise_v = move->cruise_v;
    // Determine the effective accel and decel
    accel_decel->effective_accel = accel->effective_accel;
    accel_decel->effective_decel = decel->effective_accel;
    // Determine time spent in each portion of move (time is the
    // distance divided by average velocity)
    accel_decel->accel_t = accel->accel_t;
    accel_decel->accel_offset_t = accel->accel_offset_t;
    accel_decel->total_accel_t = accel->total_accel_t;
    accel_decel->uncomp_accel_t = accel->uncomp_accel_t;
    accel_decel->uncomp_accel_offset_t = accel->uncomp_accel_offset_t;
    accel_decel->decel_t = decel->accel_t;
    accel_decel->decel_offset_t = decel->accel_offset_t;
    accel_decel->total_decel_t = decel->total_accel_t;
    accel_decel->uncomp_decel_t = decel->uncomp_accel_t;
    accel_decel->uncomp_decel_offset_t = decel->uncomp_accel_offset_t;
    double cruise_d = move->move_d - accel->accel_d - decel->accel_d;
    accel_decel->cruise_t = cruise_d / move->cruise_v;
    // Only used to track smootheness, can be deleted
    double start_v, end_v;
    if (accel_decel->accel_t)
        start_v = accel_decel->start_accel_v + accel_decel->effective_accel * accel_decel->accel_offset_t;
    else
        start_v = move->cruise_v - accel_decel->effective_decel * accel_decel->decel_offset_t;
    if (accel_decel->decel_t || accel_decel->cruise_t)
        end_v = move->cruise_v - accel_decel->effective_decel * (accel_decel->decel_offset_t + accel_decel->decel_t);
    else
        end_v = start_v + accel_decel->effective_accel * accel_decel->accel_t;
    // errorf("Move ms_v=%.3f, mc_v=%.3f, me_v=%.3f with"
    //        " move_d=%.6f, max_c_v2=%.3f, jct_v2=%.3f, accel=%.3f, decel=%.3f"
    //        ", accel_t=%.3f, cruise_t=%.3f, decel_t=%.3f"
    //        , start_v, move->cruise_v, end_v, move->move_d
    //        , move->max_cruise_v2, move->junction_max_v2
    //        , accel_decel->effective_accel, accel_decel->effective_decel
    //        , accel_decel->accel_t, accel_decel->cruise_t, accel_decel->decel_t);
    if (accel_decel->cruise_t < -EPSILON) {
        errorf("Logic error: impossible move ms_v=%.3f, mc_v=%.3f"
                ", me_v=%.3f, accel_d = %.3f, decel_d = %.3f"
                " with move_d=%.3lf, accel=%.3f, decel=%.3f"
                ", jerk=%.3f", start_v, move->cruise_v, end_v
                    , accel->accel_d, decel->accel_d, move->move_d
                    , accel->max_accel, decel->max_accel
                    , accel->max_jerk);
        return ERROR_RET;
    }
    accel_decel->cruise_t = MAX(0., accel_decel->cruise_t);
    if (fabs(mq->prev_move_end_v - start_v) > 0.0001) {
        errorf("Logic error: velocity jump from %.6f to %.6f"
                , mq->prev_move_end_v, start_v);
        return ERROR_RET;
    }
    // Remove processed move from the queue
    list_del(&move->node);
    free(move);
    mq->prev_move_end_v = end_v;
    return accel_decel->accel_t + accel_decel->cruise_t + accel_decel->decel_t;
}

int __visible
moveq_plan(struct moveq *mq, int lazy)
{
    if (list_empty(&mq->moves))
        return 0;
    double flush_starttime = get_monotonic();
    int ret;
    struct qmove *flush_limit = backward_smoothed_pass(mq, lazy, &ret);
    if (ret) return ret;
    if (lazy && !flush_limit)
        return 0;
    backward_pass(mq);
    flush_limit = compute_safe_flush_limit(mq, lazy, flush_limit);
    struct qmove *last_flushed_move = forward_pass(mq, flush_limit, lazy);
    if (!last_flushed_move)
        return 0;
    mq->prev_end_v2 = last_flushed_move->decel_group.max_start_v2;
    struct qmove *move = NULL;
    int flush_count = 0;
    list_for_each_entry(move, &mq->moves, node) {
        ++flush_count;
        if (move == last_flushed_move) break;
    }
    int qsize = 0;
    list_for_each_entry(move, &mq->moves, node)
        ++qsize;
    double flush_endtime = get_monotonic();
    errorf("lazy = %d, qsize = %d, flush_count = %d, flush_time = %.6f"
            , lazy, qsize, flush_count, flush_endtime-flush_starttime);
    return flush_count;
}
