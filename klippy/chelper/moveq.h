#ifndef MOVEQ_H
#define MOVEQ_H

#include "itersolve.h"
#include "list.h"

#define ERROR_RET -1

struct junction_point {
    struct list_node node;
    struct accel_group accel;
    struct accel_group *ma;
    double min_start_time, min_end_time;
};

struct moveq {
    double prev_end_v2;
    struct list_head moves;
    struct list_head junctions;
    double junct_start_v2;
    struct move *smoothed_pass_limit;
    double prev_move_end_v;
};

struct moveq *moveq_alloc(void);
void moveq_reset(struct moveq *mq);

int moveq_add(struct moveq *mq, int is_kinematic_move, double move_d
              , double start_pos_x, double start_pos_y, double start_pos_z
              , double axes_d_x, double axes_d_y, double axes_d_z
              , double start_pos_e, double axes_d_e
              , double junction_max_v2, double velocity
              , int accel_order, double accel, double smoothed_accel
              , double jerk, double min_jerk_limit_time, double accel_comp);
int moveq_flush(struct moveq *mq, int lazy);
double moveq_getmove(struct moveq *mq, double print_time, struct move *m);

#endif  // moveq.h
