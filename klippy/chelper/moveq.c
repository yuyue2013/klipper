#include <assert.h> // assert
#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h"
#include "moveq.h"
#include "pyhelper.h" // get_monotonic

static const double EPSILON = 0.000000001;

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static void
reset_junctions(struct moveq *mq, double start_v2)
{
    while (likely(!list_empty(&mq->junctions))) {
        struct junction_point *jp = list_first_entry(
                &mq->junctions, struct junction_point, node);
        list_del(&jp->node);
        free(jp);
    }
    mq->junct_start_v2 = start_v2;
}

static struct junction_point *
junction_point_alloc(void)
{
    struct junction_point *jp = malloc(sizeof(*jp));
    memset(jp, 0, sizeof(*jp));
    return jp;
}

struct moveq * __visible
moveq_alloc(void)
{
    struct moveq *mq = malloc(sizeof(*mq));
    memset(mq, 0, sizeof(*mq));
    list_init(&mq->moves);
    list_init(&mq->junctions);
    return mq;
}

void __visible
moveq_reset(struct moveq *mq)
{
    struct move *m = NULL, *nm = NULL;
    list_for_each_entry_safe(m, nm, &mq->moves, node) {
        list_del(&m->node);
        free(m);
    }
    struct junction_point *jp = NULL, *njp = NULL;
    list_for_each_entry_safe(jp, njp, &mq->junctions, node) {
        list_del(&jp->node);
        free(jp);
    }
    memset(mq, 0, sizeof(*mq));
    list_init(&mq->moves);
    list_init(&mq->junctions);
}

static void
fill_accel_group(struct accel_group *ag, struct move *m, double accel
                 , double jerk, double min_jerk_limit_time)
{
    ag->max_accel = accel;
    ag->max_jerk = jerk;
    ag->min_jerk_limit_time = min_jerk_limit_time;
    ag->min_accel = jerk * min_jerk_limit_time / 6.;
    if (unlikely(ag->min_accel > ag->max_accel)) {
        ag->min_accel = ag->max_accel;
    }
    ag->move = m;
}

inline static void
limit_accel(struct accel_group *ag, double accel, double jerk)
{
    if (accel < 0.) accel = 0.;
    ag->max_accel = MIN(ag->max_accel, accel);
    ag->max_jerk = MIN(ag->max_jerk, jerk);
    double min_accel = ag->max_jerk * ag->min_jerk_limit_time / 6.;
    if (unlikely(ag->min_accel > min_accel)) {
        ag->min_accel = min_accel;
    }
    if (unlikely(ag->min_accel > ag->max_accel)) {
        ag->min_accel = ag->max_accel;
    }
}

inline static void
set_max_start_v2(struct accel_group *ag, double start_v2)
{
    ag->max_start_v2 = start_v2;
    ag->max_start_v = sqrt(start_v2);
}

inline static double
calc_max_v2(const struct accel_group* ag)
{
    double dist = ag->combined_d;
    // Check if accel is the limiting factor
    double start_v2 = ag->start_accel->max_start_v2;
    double max_accel_v2 = start_v2 + 2.0 * dist * ag->max_accel;
    if (unlikely(ag->move->accel_order == 2))
        return max_accel_v2;
    // Compute maximum achievable speed with limited kinematic jerk using
    // max(jerk) == 6 * accel / accel_time, which is exact for accel order 4
    // and is quite accurate for accel order 6:
    // max(jerk) == 10 / sqrt(3) * accel / accel_time ~=
    //     5.774 * accel / accel_time
    // This leads to the cubic equation
    // (max_v^2 - start_v^2) * (max_v + start_v) / 2 ==
    //     dist^2 * jerk / 3
    // which is solved using Cardano's formula.
    double start_v = ag->start_accel->max_start_v;
    double a = 2./3. * start_v;
    double b = a*a*a;
    double c = dist * dist * ag->max_jerk / 3.;
    double max_v;
    if (unlikely(b * 54 < c)) {
        // Make max_v monotonic over start_v: return the max velocity
        // which works for any start_v velocity below the threshold.
        // Combine algorithm relies on monotonicity of max_v(start_v).
        max_v = 1.5 * pow(c*.5, 1./3.);
    } else {
        double d = sqrt(c * (c + 2. * b));
        double e = pow(b + c + d, 1./3.);
        if (e < EPSILON)
            return start_v;
        max_v = e + a*a / e - start_v / 3.;
    }
    double max_v2 = max_v * max_v;
    if (unlikely(max_accel_v2 < max_v2))
        max_v2 = max_accel_v2;
    double min_accel_v2 = start_v2 + 2.0 * dist * ag->min_accel;
    if (unlikely(min_accel_v2 > max_v2))
        max_v2 = min_accel_v2;
    return max_v2;
}

inline static double
calc_effective_accel(const struct accel_group *ag, double cruise_v)
{
    if (unlikely(ag->move->accel_order == 2))
        return ag->max_accel;
    double effective_accel = sqrt(ag->max_jerk
            * (cruise_v - ag->start_accel->max_start_v) / 6.);
    if (unlikely(effective_accel > ag->max_accel))
        effective_accel = ag->max_accel;
    if (unlikely(effective_accel < ag->min_accel))
        effective_accel = ag->min_accel;
    return effective_accel;
}

inline static double
calc_min_accel_time(const struct accel_group *ag, double cruise_v)
{
    double delta_v = cruise_v - ag->start_accel->max_start_v;
    double min_accel_time = delta_v / ag->max_accel;
    if (likely(ag->move->accel_order > 2)) {
        double accel_t = sqrt(6. * delta_v / ag->max_jerk);
        if (likely(accel_t > min_accel_time))
            min_accel_time = accel_t;
    }
    if (likely(ag->min_accel)) {
        double accel_t = delta_v / ag->min_accel;
        if (unlikely(accel_t < min_accel_time))
            min_accel_time = accel_t;
    }
    return min_accel_time;
}

inline static double
calc_min_accel_dist(const struct accel_group *ag, double cruise_v)
{
    double start_v = ag->start_accel->max_start_v;
    if (unlikely(cruise_v <= start_v))
        return 0.;
    double accel_t = calc_min_accel_time(ag, cruise_v);
    return (start_v + cruise_v) * 0.5 * accel_t;
}

inline static double
calc_min_safe_dist(const struct accel_group *ag, double cruise_v2)
{
    double min_dist = cruise_v2 / (2.0 * ag->max_accel);
    if (likely(ag->move->accel_order > 2)) {
        // It is possible to decelerate from cruise_v2 to any other velocity
        // in range [0; cruise_v2]. If deceleration distance is smaller than
        // this, some velocities in that range are prohibited for deceleration.
        double d = sqrt((16. / 9.) * pow(cruise_v2, 1.5) / ag->max_jerk);
        min_dist = MAX(min_dist, d);
    }
    return min_dist;
}

inline static void
calc_min_accel_end_time(struct junction_point *jp, double cruise_v2)
{
    if (jp->accel.start_accel->max_start_v2 >= cruise_v2) {
        jp->min_end_time = jp->accel.combined_d / cruise_v2;
    } else {
        double start_v = jp->accel.start_accel->max_start_v;
        double cruise_v = sqrt(cruise_v2);
        double accel_t = calc_min_accel_time(&jp->accel, cruise_v);
        double cruise_t = (jp->accel.combined_d
                - (start_v + cruise_v) * 0.5 * accel_t) / cruise_v;
        jp->min_end_time = accel_t + cruise_t;
    }
    jp->min_end_time += jp->min_start_time;
}

static void
process_next_accel(struct moveq *mq, struct accel_group *ag
                   , double junction_max_v2)
{
    struct junction_point *new_jp = junction_point_alloc();
    new_jp->accel = *ag;
    new_jp->accel.start_accel = &new_jp->accel;
    new_jp->ma = ag;
    struct junction_point *const prev_jp = list_empty(&mq->junctions)
        ? NULL : list_last_entry(&mq->junctions, struct junction_point, node);
    struct accel_group *prev_accel = NULL;
    const double max_cruise_v2 = ag->move->max_cruise_v2;
    double start_v2 = junction_max_v2;
    if (likely(prev_jp)) {
        prev_accel = prev_jp->ma;
        start_v2 = MIN(start_v2
                , MIN(prev_accel->max_end_v2, prev_accel->move->max_cruise_v2));
        new_jp->min_start_time = prev_jp->min_end_time;
    } else {
        start_v2 = MIN(start_v2, mq->junct_start_v2);
    }
    if (prev_accel) prev_accel->next_accel = ag;
    set_max_start_v2(&new_jp->accel, start_v2);
    if (unlikely(!prev_accel || ag->move->accel_order == 2
                || prev_accel->move->accel_order != ag->move->accel_order)) {
        reset_junctions(mq, start_v2);
    }
    const double junction_accel_limit_v2 = junction_max_v2 * (53. / 54.);
    while (likely(!list_empty(&mq->junctions))) {
        struct junction_point *last_jp = list_last_entry(
                &mq->junctions, struct junction_point, node);
        if (unlikely(last_jp->accel.max_start_v2 + EPSILON
                    < MIN(start_v2, junction_accel_limit_v2))) {
            // First point from which acceleration is possible
            break;
        }
        // This point requires deceleration and not acceleration
        list_del(&last_jp->node);
        free(last_jp);
    }
    struct junction_point *jp = NULL;
    list_for_each_entry(jp, &mq->junctions, node) {
        // Make sure to not exceed junction_max_v2 during acceleration
        // During S-Curve acceleration, the actual speed can overshoot
        // (start_v + accel * t) by (accel * t / (6 * sqrt(3)))
        const double junction_accel_limit = 0.5 * (junction_accel_limit_v2
                - jp->accel.max_start_v2) / jp->accel.combined_d;
        limit_accel(&jp->accel, MIN(junction_accel_limit, ag->max_accel)
                , ag->max_jerk);
    }
    // Add the current move to the list (with combined_d == 0)
    list_add_tail(&new_jp->node, &mq->junctions);

    struct junction_point *best_jp = NULL;
    list_for_each_entry(jp, &mq->junctions, node) {
        // Choose the best acceleration option
        jp->accel.combined_d += ag->move->move_d;
        jp->accel.max_end_v2 = calc_max_v2(&jp->accel);
        calc_min_accel_end_time(jp, MIN(jp->accel.max_end_v2, max_cruise_v2));
        if (!best_jp || best_jp->min_end_time > jp->min_end_time)
            best_jp = jp;
    }
    assert(best_jp);
    limit_accel(ag, best_jp->accel.max_accel, best_jp->accel.max_jerk);
    set_max_start_v2(ag, start_v2);
    ag->max_end_v2 = best_jp->accel.max_end_v2;
    ag->combined_d = best_jp->accel.combined_d;
    // Point to the real accel_group instance.
    ag->start_accel = best_jp->ma;
}

static int
set_accel(struct accel_group* combined, double cruise_v2
          , int time_offset_from_start)
{
    if (combined->start_accel->max_start_v2 > cruise_v2)
        set_max_start_v2(combined->start_accel, cruise_v2);
    double start_accel_v = combined->start_accel->max_start_v;
    double cruise_v = sqrt(cruise_v2);
    double avg_v = (cruise_v + start_accel_v) * 0.5;
    double combined_accel_t = calc_min_accel_time(combined, cruise_v);
    double combined_accel_d = avg_v * combined_accel_t;
    if (combined_accel_d > combined->combined_d + EPSILON) {
        errorf("Logic error: need %.6f to accelerate, only %.6f combined"
                , combined_accel_d, combined->combined_d);
        return ERROR_RET;
    }
    double effective_accel = calc_effective_accel(combined, cruise_v);
    struct move m;
    memset(&m, 0, sizeof(m));
    m.accel_order = combined->move->accel_order;
    move_fill(&m, 0.,
        combined_accel_t, 0., combined_accel_t,
        0.,
        0., 0., 0.,
        start_accel_v, cruise_v, effective_accel, 0.);
    double remaining_accel_t = combined_accel_t;
    double remaining_accel_d = combined_accel_d;
    struct accel_group *a = combined->start_accel;
    for (;;) {
        a->move->cruise_v = cruise_v;
        if (likely(remaining_accel_d > 0)) {
            a->effective_accel = effective_accel;
            a->total_accel_t = combined_accel_t;
            a->accel_d = MIN(a->move->move_d, remaining_accel_d);
            a->start_accel_v = start_accel_v;
            double next_pos = a->accel_d + combined_accel_d - remaining_accel_d;
            if (time_offset_from_start) {
                a->accel_offset_t = combined_accel_t - remaining_accel_t;
                a->accel_t = move_get_time(&m, next_pos) - a->accel_offset_t;
            } else {
                a->accel_offset_t =
                    combined_accel_t - move_get_time(&m, next_pos);
                a->accel_t = remaining_accel_t - a->accel_offset_t;
            }
            remaining_accel_t -= a->accel_t;
            remaining_accel_d -= a->move->move_d;
        }
        if (unlikely(a == combined)) break;
        a = a->next_accel;
    }
    return 0;
}

static double
calc_move_peak_v2(struct move *m)
{
    struct accel_group *accel = &m->accel_group;
    struct accel_group *decel = &m->decel_group;
    if (m->accel_order == 2) {
        double effective_accel = MIN(accel->max_accel, decel->max_accel);
        double peak_v2 = (accel->max_start_v2 + decel->max_start_v2
                + 2. * m->move_d * effective_accel) * .5;
        return peak_v2;
    }
    double total_d = accel->combined_d + decel->combined_d - m->move_d;
    double high_v = sqrt(MAX(accel->max_end_v2, decel->max_end_v2));
    double low_v = 0.;
    while (likely(high_v - low_v > EPSILON)) {
        double guess_v = (high_v + low_v) * 0.5;
        double accel_d = calc_min_accel_dist(accel, guess_v);
        double decel_d = calc_min_accel_dist(decel, guess_v);
        if (accel_d <= accel->combined_d && decel_d <= decel->combined_d &&
                accel_d + decel_d <= total_d)
            low_v = guess_v;
        else
            high_v = guess_v;
    }
    return low_v * low_v;
}

static struct move *
backward_pass(struct moveq *mq, int lazy, int *ret)
{
    *ret = 0;
    int update_flush_limit = lazy;
    // Traverse queue from last to first move and determine maximum
    // junction speed assuming the robot comes to a complete stop
    // after the last move.
    struct list_head delayed;
    list_init(&delayed);
    double next_smoothed_v2 = 0., peak_cruise_v2 = 0., junction_max_v2 = 0.;
    reset_junctions(mq, 0.);
    struct move *move = NULL, *pm = NULL, *flush_limit = NULL;
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
            struct move *m = NULL;
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
                    m->junction_max_v2 = MIN(m->junction_max_v2, peak_cruise_v2);
                }
            }
            struct move *nm = NULL, *qm = move;
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

    list_for_each_entry_reversed(move, &mq->moves, node) {
        // Restore the default accel and decel values if they were modified
        // on previous backward pass
        move->decel_group = move->accel_group = move->default_accel;

        // Backward pass for full acceleration
        struct accel_group *decel = &move->decel_group;
        process_next_accel(mq, decel, junction_max_v2);
        junction_max_v2 = move->junction_max_v2;
    }
    if (!lazy) return flush_limit;
    for (move = flush_limit;
            !list_at_end(move, &mq->moves, node);
            move = list_prev_entry(move, node)) {
        struct accel_group safe_decel = move->decel_group;
        safe_decel.combined_d = 0.;
        struct move *m, *nm = NULL;
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

static double
calc_trap_peak_v2(struct move *accel_head, struct move *decel_head)
{
    if (decel_head != accel_head) {
        double peak_v2 = MIN(decel_head->decel_group.max_end_v2
                , decel_head->junction_max_v2);
        if (accel_head)
            peak_v2 = MIN(peak_v2, accel_head->accel_group.max_end_v2);
        return peak_v2;
    }
    double peak_v2 = calc_move_peak_v2(decel_head);
    return MIN(peak_v2, decel_head->max_cruise_v2);
}

static int
set_trap_decel(struct move *decel_head, struct list_head *trapezoid
               , double cruise_v2)
{
    struct move *m = decel_head;
    while (!list_at_end(m, trapezoid, node)) {
        int ret = set_accel(&m->decel_group, cruise_v2
                , /*time_offset_from_start=*/0);
        if (ret) return ret;
        m = m->decel_group.start_accel->move;
        cruise_v2 = MIN(cruise_v2, m->decel_group.max_start_v2);
        m = list_next_entry(m, node);
    }
    return 0;
}

static int
set_trap_accel(struct move *accel_head, struct list_head *trapezoid
               , double cruise_v2)
{
    struct move *m = accel_head;
    while (!list_at_end(m, trapezoid, node)) {
        int ret = set_accel(&m->accel_group, cruise_v2
                , /*time_offset_from_start=*/1);
        if (ret) return ret;
        m = m->accel_group.start_accel->move;
        cruise_v2 = MIN(cruise_v2, m->accel_group.max_start_v2);
        m = list_prev_entry(m, node);
    }
    return 0;
}

static struct move *
trap_flush(struct list_node *next_pos
           , struct list_head *trapezoid , struct move **accel_head
           , struct move **decel_head, int *ret)
{
    double peak_cruise_v2 = calc_trap_peak_v2(*accel_head, *decel_head);
    *ret = 0;
    if (*decel_head)
        *ret = set_trap_decel(*decel_head, trapezoid, peak_cruise_v2);
    if (*ret) return NULL;
    if (*accel_head)
        *ret = set_trap_accel(*accel_head, trapezoid, peak_cruise_v2);
    if (*ret) return NULL;
    struct move *move = NULL, *next = NULL, *prev = NULL;
    list_for_each_entry_safe(move, next, trapezoid, node) {
        list_del(&move->node);
        list_add_before(&move->node, next_pos);
        prev = move;
    }
    *accel_head = *decel_head = NULL;
    return prev;
}

static void
add_as_accel(struct move *move, struct list_head *trapezoid
             , struct move **accel_head, struct move **decel_head)
{
    list_del(&move->node);
    list_add_tail(&move->node, trapezoid);
    *accel_head = move;
}

static void
add_as_decel(struct move *move, struct list_head *trapezoid
             , struct move **accel_head, struct move **decel_head)
{
    if (!*decel_head)
        *decel_head = move;
    if (*accel_head != move) {
        list_del(&move->node);
        list_add_tail(&move->node, trapezoid);
    }
}

static struct move *
forward_pass(struct moveq *mq, struct move *end, int lazy, int *ret)
{
    *ret = 0;
    struct move *move = list_first_entry(&mq->moves, struct move, node);
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
    struct list_head trapezoid;
    list_init(&trapezoid);
    struct move *accel_head = NULL, *decel_head = NULL;

    reset_junctions(mq, start_v2);
    double prev_cruise_v2 = start_v2;
    struct move *last_flushed_move = NULL, *next_move = NULL;
    for (; !list_at_end(move, &mq->moves, node) && move != end;
            move = next_move) {
        // Track next_move early because move will be moved to trapezoid list
        next_move = list_next_entry(move, node);
        struct accel_group *accel = &move->accel_group;
        struct accel_group *decel = &move->decel_group;

        process_next_accel(mq, accel
                , MIN(move->junction_max_v2, prev_cruise_v2));
        int can_accelerate = decel->max_end_v2 > accel->max_start_v2 + EPSILON;
        if (can_accelerate) {
            // This move can accelerate
            if (decel_head) {
                last_flushed_move = trap_flush(&move->node, &trapezoid
                        , &accel_head, &decel_head, ret);
                if (*ret) return NULL;
            }
            add_as_accel(move, &trapezoid, &accel_head, &decel_head);
        }
        int must_decelerate = accel->max_end_v2 + EPSILON > decel->max_start_v2;
        if (must_decelerate || !can_accelerate) {
            // This move must decelerate after acceleration,
            // or this is a full decel move after full accel move.
            for (; move != end; move = next_move
                    , next_move = list_next_entry(move, node)) {
                add_as_decel(move, &trapezoid, &accel_head, &decel_head);
                if (move == decel->start_accel->move) break;
            }
            if (move == end) break;
            reset_junctions(mq, decel->start_accel->max_start_v2);
        }
        prev_cruise_v2 = move->max_cruise_v2;
    }
    if (!lazy) {
        if (decel_head)
            last_flushed_move = trap_flush(&mq->moves.root, &trapezoid
                    , &accel_head, &decel_head, ret);
        if (*ret) return NULL;
    } else {
        assert(end != NULL);
        // Just put the remaining moves back to mq->moves
        struct move *next = NULL;
        list_for_each_entry_safe(move, next, &trapezoid, node) {
            list_del(&move->node);
            list_add_before(&move->node, &end->node);
        }
    }
    return last_flushed_move;
}

int __visible
moveq_add(struct moveq *mq, int is_kinematic_move, double move_d
          , double start_pos_x, double start_pos_y, double start_pos_z
          , double axes_d_x, double axes_d_y, double axes_d_z
          , double start_pos_e, double axes_d_e
          , double junction_max_v2, double velocity
          , int accel_order, double accel, double smoothed_accel
          , double jerk, double min_jerk_limit_time)
{
    struct move *m = move_alloc();

    m->accel_order = accel_order;
    m->is_kinematic_move = is_kinematic_move;
    m->move_d = move_d;

    m->start_pos.x = start_pos_x;
    m->start_pos.y = start_pos_y;
    m->start_pos.z = start_pos_z;
    m->extrude_pos = start_pos_e;
    double inv_kin_move_d = 1. / sqrt(axes_d_x*axes_d_x + axes_d_y*axes_d_y
                                      + axes_d_z*axes_d_z);
    m->axes_r.x = axes_d_x * inv_kin_move_d;
    m->axes_r.y = axes_d_y * inv_kin_move_d;
    m->axes_r.z = axes_d_z * inv_kin_move_d;
    m->extrude_d = axes_d_e;

    fill_accel_group(&m->default_accel, m, accel, jerk, min_jerk_limit_time);
    m->max_cruise_v2 = velocity * velocity;
    m->junction_max_v2 = junction_max_v2;
    m->smooth_delta_v2 = 2. * smoothed_accel * move_d;

    if (!list_empty(&mq->moves)) {
        struct move *prev_move = list_last_entry(&mq->moves, struct move, node);
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
moveq_getmove(struct moveq *mq, double print_time, struct move *m)
{
    assert(!list_empty(&mq->moves));
    struct move *move = list_first_entry(&mq->moves, struct move, node);
    struct accel_group *accel = &move->accel_group;
    struct accel_group *decel = &move->decel_group;
    // Determine move velocities
    double start_accel_v = accel->start_accel_v;
    // Determine the effective accel and decel
    double effective_accel = accel->effective_accel;
    double effective_decel = decel->effective_accel;
    // Determine time spent in each portion of move (time is the
    // distance divided by average velocity)
    double accel_t = accel->accel_t;
    double accel_offset_t = accel->accel_offset_t;
    double total_accel_t = accel->total_accel_t;
    double decel_t = decel->accel_t;
    double decel_offset_t = decel->accel_offset_t;
    double total_decel_t = decel->total_accel_t;
    double cruise_t = (move->move_d
            - accel->accel_d - decel->accel_d) / move->cruise_v;
    // Only used to track smootheness, can be deleted
    double start_v, end_v;
    if (accel_t)
        start_v = start_accel_v + effective_accel * accel_offset_t;
    else
        start_v = move->cruise_v - effective_decel * decel_offset_t;
    if (decel_t || cruise_t)
        end_v = move->cruise_v - effective_decel * (decel_offset_t + decel_t);
    else
        end_v = start_v + effective_accel * accel_t;
    // errorf("Move ms_v=%.3f, mc_v=%.3f, me_v=%.3f with"
    //        " move_d=%.6f, max_c_v2=%.3f, jct_v2=%.3f, accel=%.3f, decel=%.3f"
    //        ", accel_t=%.3f, cruise_t=%.3f, decel_t=%.3f"
    //        , start_v, move->cruise_v, end_v, move->move_d
    //        , move->max_cruise_v2, move->junction_max_v2
    //        , effective_accel, effective_decel
    //        , accel_t, cruise_t, decel_t);
    if (cruise_t < -EPSILON) {
        errorf("Logic error: impossible move ms_v=%.3f, mc_v=%.3f"
                ", me_v=%.3f, accel_d = %.3f, decel_d = %.3f"
                " with move_d=%.3lf, accel=%.3f, decel=%.3f"
                ", jerk=%.3f", start_v, move->cruise_v, end_v
                    , accel->accel_d, decel->accel_d, move->move_d
                    , accel->max_accel, decel->max_accel
                    , accel->max_jerk);
        return ERROR_RET;
    }
    if (fabs(mq->prev_move_end_v - start_v) > 0.0001) {
        errorf("Logic error: velocity jump from %.6f to %.6f"
                , mq->prev_move_end_v, start_v);
        return ERROR_RET;
    }
    // Generate step times for the move
    move_fill(move, print_time,
        accel_t, accel_offset_t, total_accel_t,
        cruise_t,
        decel_t, decel_offset_t, total_decel_t,
        start_accel_v, move->cruise_v,
        effective_accel, effective_decel);
    *m = *move;
    // Remove processed move from the queue
    list_del(&move->node);
    free(move);
    mq->prev_move_end_v = end_v;
    return accel_t + cruise_t + decel_t;
}

int __visible
moveq_flush(struct moveq *mq, int lazy)
{
    if (list_empty(&mq->moves))
        return 0;
    double flush_starttime = get_monotonic();
    int ret;
    struct move *flush_limit = backward_pass(mq, lazy, &ret);
    if (ret) return ret;
    if (lazy && !flush_limit)
        return 0;
    struct move *last_flushed_move = forward_pass(mq, flush_limit, lazy, &ret);
    if (ret) return ret;
    if (!last_flushed_move)
        return 0;
    mq->prev_end_v2 = last_flushed_move->decel_group.max_start_v2;
    struct move *move = NULL;
    int flush_count = 0;
    list_for_each_entry(move, &mq->moves, node) {
        ++flush_count;
        if (move == last_flushed_move) break;
    }
    int qsize = 0;
    list_for_each_entry(move, &mq->moves, node)
        ++qsize;
    double flush_endtime = get_monotonic();
    errorf("lazy = %d, qsize = %d, flush_count = %d, flush_time = %d"
            , lazy, qsize, flush_count
            , (int)((flush_endtime-flush_starttime)*1000000));
    return flush_count;
}
