// Moves combiner computing groups of moves that should accelerate together
//
// Copyright (C) 2019  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <assert.h> // assert
#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "accelcombine.h"
#include "accelgroup.h"
#include "compiler.h" // likely
#include "moveq.h" // qmove

static const double EPSILON = 0.000000001;

struct junction_point {
    struct list_node node;
    // Combined acceleration limits that must be respected
    // from this junction point
    struct accel_group accel;
    struct accel_group *move_ag;
    double min_start_time;
};

void
reset_junctions(struct accel_combiner *ac, double start_v2)
{
    while (likely(!list_empty(&ac->junctions))) {
        struct junction_point *jp = list_first_entry(
                &ac->junctions, struct junction_point, node);
        list_del(&jp->node);
        free(jp);
    }
    ac->junct_start_v2 = start_v2;
    ac->min_end_time = 0;
}

struct junction_point *
create_junction_point(struct accel_combiner *ac, struct accel_group *ag
                      , double junction_max_v2)
{
    struct junction_point *new_jp = malloc(sizeof(*new_jp));
    memset(new_jp, 0, sizeof(*new_jp));
    new_jp->accel = *ag;
    new_jp->accel.start_accel = &new_jp->accel;
    new_jp->move_ag = ag;
    struct junction_point *prev_jp = list_empty(&ac->junctions)
        ? NULL : list_last_entry(&ac->junctions, struct junction_point, node);
    struct accel_group *prev_accel = NULL;
    double start_v2 = junction_max_v2;
    if (likely(prev_jp)) {
        prev_accel = prev_jp->move_ag;
        start_v2 = MIN(start_v2
                , MIN(prev_accel->max_end_v2, prev_accel->move->max_cruise_v2));
        new_jp->min_start_time = ac->min_end_time;
    } else {
        start_v2 = MIN(start_v2, ac->junct_start_v2);
    }
    if (prev_accel) prev_accel->next_accel = ag;
    set_max_start_v2(&new_jp->accel, start_v2);
    return new_jp;
}

inline static int
check_can_combine(struct accel_combiner *ac
                  , struct junction_point *next_jp)
{
    if (unlikely(list_empty(&ac->junctions)))
        return 0;
    struct junction_point *prev_jp = list_last_entry(
            &ac->junctions, struct junction_point, node);
    struct accel_group *prev_accel = prev_jp->move_ag;
    struct accel_group *next_accel = next_jp->move_ag;
    return next_accel->accel_order != 2
        && prev_accel->accel_order == next_accel->accel_order
        && prev_accel->move->accel_comp == next_accel->move->accel_comp;
}

static void
drop_decelerating_jps(struct accel_combiner *ac, double accel_limit_v2)
{
    while (likely(!list_empty(&ac->junctions))) {
        struct junction_point *last_jp = list_last_entry(
                &ac->junctions, struct junction_point, node);
        if (unlikely(last_jp->accel.max_start_v2 + EPSILON
                    < accel_limit_v2)) {
            // First point from which acceleration is possible
            return;
        }
        // This point requires deceleration and not acceleration
        list_del(&last_jp->node);
        free(last_jp);
    }
}

static void
limit_accel_jps(struct accel_combiner *ac, struct accel_group *new_ag
                , double junction_max_v2)
{
    struct junction_point *jp = NULL;
    list_for_each_entry(jp, &ac->junctions, node) {
        // Make sure to not exceed junction_max_v2 during acceleration
        double junction_accel_limit = 0.5 * (junction_max_v2
                - jp->accel.max_start_v2) / jp->accel.combined_d;
        limit_accel(&jp->accel, MIN(junction_accel_limit, new_ag->max_accel)
                , new_ag->max_jerk);
    }
}

inline static double
calc_min_accel_end_time(struct junction_point *jp, double cruise_v2)
{
    return jp->min_start_time
        + calc_min_accel_group_time(&jp->accel, sqrt(cruise_v2));
}

static struct junction_point *
calc_best_jp(struct accel_combiner *ac, struct accel_group *new_ag)
{
    double max_cruise_v2 = new_ag->move->max_cruise_v2;
    struct junction_point *jp = NULL, *best_jp = NULL;
    double min_end_time = -1.;
    list_for_each_entry(jp, &ac->junctions, node) {
        // Choose the best acceleration option
        jp->accel.combined_d += new_ag->move->move_d;
        jp->accel.max_end_v2 = calc_max_v2(&jp->accel);
        double end_time = calc_min_accel_end_time(
                jp, MIN(jp->accel.max_end_v2, max_cruise_v2));
        if (!best_jp || min_end_time > end_time + EPSILON) {
            best_jp = jp;
            min_end_time = end_time;
        }
    }
    ac->min_end_time = min_end_time;
    assert(best_jp);
    return best_jp;
}

void
process_next_accel(struct accel_combiner *ac, struct accel_group *ag
                   , double junction_max_v2)
{
    struct junction_point *new_jp = create_junction_point(
            ac, ag, junction_max_v2);
    double start_v2 = new_jp->accel.max_start_v2;
    if (unlikely(!check_can_combine(ac, new_jp)))
        reset_junctions(ac, start_v2);

    drop_decelerating_jps(ac, MIN(start_v2, junction_max_v2));
    limit_accel_jps(ac, ag, junction_max_v2);

    // Add the current move to the list (with combined_d == 0)
    list_add_tail(&new_jp->node, &ac->junctions);
    struct junction_point *best_jp = calc_best_jp(ac, ag);

    limit_accel(ag, best_jp->accel.max_accel, best_jp->accel.max_jerk);
    set_max_start_v2(ag, start_v2);
    ag->max_end_v2 = best_jp->accel.max_end_v2;
    ag->combined_d = best_jp->accel.combined_d;
    // Point to the real accel_group instance.
    ag->start_accel = best_jp->move_ag;
}

void
init_combiner(struct accel_combiner *ac)
{
    list_init(&ac->junctions);
    ac->junct_start_v2 = 0;
    ac->min_end_time = 0;
}

void
reset_combiner(struct accel_combiner *ac)
{
    reset_junctions(ac, 0.);
}
