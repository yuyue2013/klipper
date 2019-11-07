#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node
#include "scurve.h"  // scurve

struct coord {
    double x, y, z;
};

struct move {
    double print_time, move_t;
    struct coord start_pos, axes_r;
    struct scurve s;

    struct list_node node;
};

struct trap_accel_decel {
    double accel_t, accel_offset_t, total_accel_t;
    double uncomp_accel_t, uncomp_accel_offset_t;
    double cruise_t;
    double decel_t, decel_offset_t, total_decel_t;
    double uncomp_decel_t, uncomp_decel_offset_t;
    double start_accel_v, cruise_v;
    double effective_accel, effective_decel;
    double accel_comp;
    int accel_order;
};

struct trapq {
    struct list_head moves;
};

struct move *move_alloc(void);
struct trap_accel_decel *accel_decel_alloc(void);
void trapq_append(struct trapq *tq, double print_time
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double axes_d_x, double axes_d_y, double axes_d_z
                  , const struct trap_accel_decel *accel_decel);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_free_moves(struct trapq *tq, double print_time);

#endif // trapq.h
