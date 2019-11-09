#ifndef ITERSOLVE_H
#define ITERSOLVE_H

#include <stdint.h> // uint32_t

struct coord {
    double x, y, z;
};

struct move_accel {
    double c0, c1, c2, c3, c4, c5, c6;
    double offset_t;
};

struct move {
    double print_time, move_t;
    double accel_t, cruise_t;
    double cruise_start_d, decel_start_d;
    double cruise_v;
    struct move_accel accel, decel;
    struct coord start_pos, axes_r;
    int accel_order;
};

struct move *move_alloc(void);
void move_set_accel_order(struct move *m, int accel_order);
void move_fill(struct move *m, double print_time
               , double accel_t, double accel_offset_t, double total_accel_t
               , double cruise_t
               , double decel_t, double decel_offset_t, double total_decel_t
               , double start_pos_x, double start_pos_y, double start_pos_z
               , double axes_d_x, double axes_d_y, double axes_d_z
               , double start_accel_v, double cruise_v
               , double accel, double decel);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
double move_get_time(struct move *m, double move_distance);

struct stepper_kinematics;
typedef double (*sk_callback)(struct stepper_kinematics *sk, struct move *m
                              , double move_time);
struct stepper_kinematics {
    double step_dist, commanded_pos;
    struct stepcompress *sc;
    sk_callback calc_position;
};

int32_t itersolve_gen_steps(struct stepper_kinematics *sk, struct move *m);
void itersolve_set_stepcompress(struct stepper_kinematics *sk
                                , struct stepcompress *sc, double step_dist);
double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
                                          , double x, double y, double z);
void itersolve_set_commanded_pos(struct stepper_kinematics *sk, double pos);
double itersolve_get_commanded_pos(struct stepper_kinematics *sk);

// Find the distance travel during acceleration/deceleration
static inline double
move_eval_accel(struct move_accel *ma, double move_time)
{
    move_time += ma->offset_t;
    double v = ma->c6;
    v = ma->c5 + v * move_time;
    v = ma->c4 + v * move_time;
    v = ma->c3 + v * move_time;
    v = ma->c2 + v * move_time;
    v = ma->c1 + v * move_time;
    return v * move_time + ma->c0;
}

#endif // itersolve.h
