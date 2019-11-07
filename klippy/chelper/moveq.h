#ifndef MOVEQ_H
#define MOVEQ_H

#include "itersolve.h"
#include "list.h"

#define ERROR_RET -1

struct qmove;
struct trap_accel_decel;

struct moveq {
    double prev_end_v2;
    struct list_head moves;
    struct list_head junctions;
    double junct_start_v2;
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
