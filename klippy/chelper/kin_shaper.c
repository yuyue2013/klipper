// Kinematic input shapers to minimize motion vibrations in XY plane
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt, exp
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

/****************************************************************
 * Generic position calculation via shaper convolution
 ****************************************************************/

static inline double
move_get_axis_coord(struct move *m, int axis, double move_time)
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_pos = m->start_pos.axis[axis - 'x'];
    double move_dist = move_get_distance(m, move_time);
    return start_pos + axis_r * move_dist;
}

struct shaper_pulse {
    double t, a;
};

// Calculate the position from the convolution of the shaper with input signal
static inline double
calc_position(struct move *m, int axis, double move_time
              , struct shaper_pulse *pulses, int n)
{
    double time = move_time + pulses[0].t;
    while (unlikely(time < 0.)) {
        m = list_prev_entry(m, node);
        time += m->move_t;
    }
    double res = 0.;
    for (int i = 0; ; ++i) {
        res += pulses[i].a * move_get_axis_coord(m, axis, time);
        if (i + 1 >= n)
            break;
        time += pulses[i+1].t - pulses[i].t;
        while (unlikely(time > m->move_t)) {
            time -= m->move_t;
            m = list_next_entry(m, node);
        }
    }
    return res;
}

/****************************************************************
 * Shaper-specific initialization
 ****************************************************************/

#define EI_SHAPER_VIB_TOL 0.05

enum INPUT_SHAPER_TYPE {
    INPUT_SHAPER_ZV = 0,
    INPUT_SHAPER_ZVD = 1,
    INPUT_SHAPER_ZVDD = 2,
    INPUT_SHAPER_ZVDDD = 3,
    INPUT_SHAPER_EI = 4,
    INPUT_SHAPER_2HUMP_EI = 5,
};

struct input_shaper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    struct shaper_pulse *x_pulses, *y_pulses;
    int x_n, y_n;
};

typedef void (*is_init_shaper_callback)(double half_period, double damping_ratio
                                        , struct shaper_pulse **pulses, int *n);

static inline double
calc_ZV_K(double damping_ratio)
{
    if (likely(!damping_ratio))
        return 1.;
    return exp(-damping_ratio * M_PI / sqrt(1. - damping_ratio*damping_ratio));
}

static void
init_shaper_zv(double half_period, double damping_ratio
               , struct shaper_pulse **pulses, int *n)
{
    *n = 2;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double inv_D = 1. / (1. + K);

    (*pulses)[0].t = -.5 * half_period;
    (*pulses)[1].t = .5 * half_period;

    (*pulses)[0].a = K * inv_D;
    (*pulses)[1].a = inv_D;
}

static void
init_shaper_zvd(double half_period, double damping_ratio
                , struct shaper_pulse **pulses, int *n)
{
    *n = 3;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double inv_D = 1. / (K2 + 2. * K + 1.);

    (*pulses)[0].t = -half_period;
    (*pulses)[1].t = 0.;
    (*pulses)[2].t = half_period;

    (*pulses)[0].a = K2 * inv_D;
    (*pulses)[1].a = 2. * K * inv_D;
    (*pulses)[2].a = inv_D;
}

static void
init_shaper_zvdd(double half_period, double damping_ratio
                 , struct shaper_pulse **pulses, int *n)
{
    *n = 4;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double K3 = K2 * K;
    double inv_D = 1. / (K3 + 3. * K2 + 3. * K + 1);

    (*pulses)[0].t = -1.5 * half_period;
    (*pulses)[1].t = -.5 * half_period;
    (*pulses)[2].t = .5 * half_period;
    (*pulses)[3].t = 1.5 * half_period;

    (*pulses)[0].a = K3 * inv_D;
    (*pulses)[1].a = 3. * K2 * inv_D;
    (*pulses)[2].a = 3. * K * inv_D;
    (*pulses)[3].a = inv_D;
}

static void
init_shaper_zvddd(double half_period, double damping_ratio
                 , struct shaper_pulse **pulses, int *n)
{
    *n = 5;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double K3 = K2 * K;
    double K4 = K3 * K;
    double inv_D = 1. / (K4 + 4. * K3 + 6. * K2 + 4. * K + 1.);

    (*pulses)[0].t = -2. * half_period;
    (*pulses)[1].t = -1. * half_period;
    (*pulses)[2].t = 0.;
    (*pulses)[3].t = 1. * half_period;
    (*pulses)[4].t = 2. * half_period;

    (*pulses)[0].a = K4 * inv_D;
    (*pulses)[1].a = 4. * K3 * inv_D;
    (*pulses)[2].a = 6. * K2 * inv_D;
    (*pulses)[3].a = 4. * K * inv_D;
    (*pulses)[4].a = inv_D;
}

static void
init_shaper_ei(double half_period, double damping_ratio
               , struct shaper_pulse **pulses, int *n)
{
    *n = 3;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double k = exp(-M_PI * damping_ratio);
    double a2 = 2. * (1. - EI_SHAPER_VIB_TOL) / (1. + EI_SHAPER_VIB_TOL) * k;
    double a3 = k * k;
    double inv_D = 1. / (1. + a2 + a3);

    (*pulses)[0].t = -half_period;
    (*pulses)[1].t = 0.;
    (*pulses)[2].t = half_period;

    (*pulses)[0].a = a3 * inv_D;
    (*pulses)[1].a = a2 * inv_D;
    (*pulses)[2].a = inv_D;
}

static void
init_shaper_2hump_ei(double half_period, double damping_ratio
                     , struct shaper_pulse **pulses, int *n)
{
    *n = 4;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double d_r = damping_ratio;
    double d_r2 = d_r * d_r;
    double d_r3 = d_r2 * d_r;

    // Coefficients calculated for 5% vibration tolerance
    double t1 = -.75;
    double t2 = 0.49890 - .75 + 0.16270 * d_r - 0.54262 * d_r2 + 6.16180 * d_r3;
    double t3 = 0.99748 - .75 + 0.18382 * d_r - 1.58270 * d_r2 + 8.17120 * d_r3;
    double t4 = 1.49920 - .75 - 0.09297 * d_r - 0.28338 * d_r2 + 1.85710 * d_r3;

    double a1 = 0.16054 + 0.76699 * d_r + 2.26560 * d_r2 - 1.22750 * d_r3;
    double a2 = 0.33911 + 0.45081 * d_r - 2.58080 * d_r2 + 1.73650 * d_r3;
    double a3 = 0.34089 - 0.61533 * d_r - 0.68765 * d_r2 + 0.42261 * d_r3;
    double a4 = 0.15997 - 0.60246 * d_r + 1.00280 * d_r2 - 0.93145 * d_r3;

    // Coefficients aN should normally sum up to 1. already, but since we use
    // a polynomial expansion, they can get slightly off. So it is better to
    // re-normalize them to avoid tiny potential print scaling problems.
    double inv_D = 1. / (a1 + a2 + a3 + a4);

    (*pulses)[0].t = -2. * half_period * t4;
    (*pulses)[1].t = -2. * half_period * t3;
    (*pulses)[2].t = -2. * half_period * t2;
    (*pulses)[3].t = -2. * half_period * t1;

    (*pulses)[0].a = a4 * inv_D;
    (*pulses)[1].a = a3 * inv_D;
    (*pulses)[2].a = a2 * inv_D;
    (*pulses)[3].a = a1 * inv_D;
}

/****************************************************************
 * Kinematics-related shaper code
 ****************************************************************/

#define DUMMY_T 500.0

// Optimized calc_position when only x axis is needed
static double
shaper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->x_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.x = calc_position(m, 'x', move_time, is->x_pulses, is->x_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// Optimized calc_position when only y axis is needed
static double
shaper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->y_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.y = calc_position(m, 'y', move_time, is->y_pulses, is->y_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// General calc_position for both x and y axes
static double
shaper_xy_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->x_n && !is->y_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos = move_get_coord(m, move_time);
    if (is->x_n)
        is->m.start_pos.x = calc_position(m, 'x', move_time
                                          , is->x_pulses, is->x_n);
    if (is->y_n)
        is->m.start_pos.y = calc_position(m, 'y', move_time
                                          , is->y_pulses, is->y_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

static void
shaper_note_generation_time(struct input_shaper *is)
{
    double pre_active = 0., post_active = 0.;
    if (is->sk.active_flags & AF_X) {
        pre_active = -is->x_pulses[0].t;
        post_active = is->x_pulses[is->x_n-1].t;
    }
    if (is->sk.active_flags & AF_Y) {
        pre_active = -is->y_pulses[0].t > pre_active
            ? -is->y_pulses[0].t : pre_active;
        post_active = is->y_pulses[is->y_n-1].t > post_active
            ? is->y_pulses[is->y_n-1].t : post_active;
    }
    is->sk.gen_steps_pre_active = pre_active;
    is->sk.gen_steps_post_active = post_active;
}

int __visible
input_shaper_set_sk(struct stepper_kinematics *sk
                    , struct stepper_kinematics *orig_sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    int af = orig_sk->active_flags & (AF_X | AF_Y);
    if (af == (AF_X | AF_Y))
        is->sk.calc_position_cb = shaper_xy_calc_position;
    else if (af & AF_X)
        is->sk.calc_position_cb = shaper_x_calc_position;
    else if (af & AF_Y)
        is->sk.calc_position_cb = shaper_y_calc_position;
    else
        return -1;
    is->sk.active_flags = orig_sk->active_flags;
    is->orig_sk = orig_sk;
    return 0;
}

static is_init_shaper_callback init_shaper_callbacks[] = {
    [INPUT_SHAPER_ZV] = &init_shaper_zv,
    [INPUT_SHAPER_ZVD] = &init_shaper_zvd,
    [INPUT_SHAPER_ZVDD] = &init_shaper_zvdd,
    [INPUT_SHAPER_ZVDDD] = &init_shaper_zvddd,
    [INPUT_SHAPER_EI] = &init_shaper_ei,
    [INPUT_SHAPER_2HUMP_EI] = &init_shaper_2hump_ei,
};

int __visible
input_shaper_set_shaper_params(struct stepper_kinematics *sk
                               , double damped_spring_period_x
                               , double damped_spring_period_y
                               , double damping_ratio_x
                               , double damping_ratio_y
                               , int shaper_type)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);

    if (shaper_type >= ARRAY_SIZE(init_shaper_callbacks) || shaper_type < 0)
        return -1;
    is_init_shaper_callback init_shaper_cb = init_shaper_callbacks[shaper_type];

    int af = is->orig_sk->active_flags & (AF_X | AF_Y);
    if (af & AF_X) {
        free(is->x_pulses);
        init_shaper_cb(.5 * damped_spring_period_x, damping_ratio_x,
                       &is->x_pulses, &is->x_n);
    }
    if (af & AF_Y) {
        free(is->y_pulses);
        init_shaper_cb(.5 * damped_spring_period_y, damping_ratio_y,
                       &is->y_pulses, &is->y_n);
    }
    shaper_note_generation_time(is);
    return 0;
}

struct stepper_kinematics * __visible
input_shaper_alloc(void)
{
    struct input_shaper *is = malloc(sizeof(*is));
    memset(is, 0, sizeof(*is));
    is->m.move_t = 2. * DUMMY_T;
    return &is->sk;
}
