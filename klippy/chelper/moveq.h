#ifndef MOVEQ_H
#define MOVEQ_H

#include "accelcombine.h"
#include "accelgroup.h"
#include "itersolve.h"
#include "list.h"

#define ERROR_RET -1

struct trap_accel_decel;

struct qmove {
    struct list_node node;

    double cruise_v;
    double move_d;
    double accel_comp;

    struct accel_group accel_group, decel_group, fallback_decel, default_accel;
    double smooth_delta_v2, max_smoothed_v2;
    double max_cruise_v2, junction_max_v2;

    // Only used to track smootheness, can be deleted
    double start_v, end_v;
};

struct moveq {
    double prev_end_v2;
    struct list_head moves;
    struct accel_combiner accel_combiner;
    struct qmove *smoothed_pass_limit;
    double prev_move_end_v;
};

struct moveq *moveq_alloc(void);
void moveq_reset(struct moveq *mq);

int moveq_add(struct moveq *mq, double move_d
              , double junction_max_v2, double velocity
              , int accel_order, double accel, double smoothed_accel
              , double jerk, double min_jerk_limit_time, double accel_comp);
int moveq_plan(struct moveq *mq, int lazy);
double moveq_getmove(struct moveq *mq, struct trap_accel_decel *accel_decel);

#endif  // moveq.h
