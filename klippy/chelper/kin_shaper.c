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

struct shaper_pulse {
    double t, a;
};

struct input_shaper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct shaper_pulse *pulses;
    int n;
};

typedef void (*is_init_shaper_callback)(struct input_shaper *is
                                        , double half_period
                                        , double damping_ratio);

static inline double
calc_ZV_K(double damping_ratio)
{
    if (likely(!damping_ratio))
        return 1.;
    return exp(-damping_ratio * M_PI / sqrt(1. - damping_ratio*damping_ratio));
}

static void
init_shaper_zv(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 2;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double inv_D = 1. / (1. + K);

    is->pulses[0].t = -.5 * half_period;
    is->pulses[1].t = .5 * half_period;

    is->pulses[0].a = K * inv_D;
    is->pulses[1].a = inv_D;
}

static void
init_shaper_zvd(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 3;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double inv_D = 1. / (K2 + 2. * K + 1.);

    is->pulses[0].t = -half_period;
    is->pulses[1].t = 0.;
    is->pulses[2].t = half_period;

    is->pulses[0].a = K2 * inv_D;
    is->pulses[1].a = 2. * K * inv_D;
    is->pulses[2].a = inv_D;
}

static void
init_shaper_zvdd(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 4;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double K3 = K2 * K;
    double inv_D = 1. / (K3 + 3. * K2 + 3. * K + 1);

    is->pulses[0].t = -1.5 * half_period;
    is->pulses[1].t = -.5 * half_period;
    is->pulses[2].t = .5 * half_period;
    is->pulses[3].t = 1.5 * half_period;

    is->pulses[0].a = K3 * inv_D;
    is->pulses[1].a = 3. * K2 * inv_D;
    is->pulses[2].a = 3. * K * inv_D;
    is->pulses[3].a = inv_D;
}

static void
init_shaper_zvddd(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 5;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double K3 = K2 * K;
    double K4 = K3 * K;
    double inv_D = 1. / (K4 + 4. * K3 + 6. * K2 + 4. * K + 1.);

    is->pulses[0].t = -2. * half_period;
    is->pulses[1].t = -1. * half_period;
    is->pulses[2].t = 0.;
    is->pulses[3].t = 1. * half_period;
    is->pulses[4].t = 2. * half_period;

    is->pulses[0].a = K4 * inv_D;
    is->pulses[1].a = 4. * K3 * inv_D;
    is->pulses[2].a = 6. * K2 * inv_D;
    is->pulses[3].a = 4. * K * inv_D;
    is->pulses[4].a = inv_D;
}

static void
init_shaper_ei(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 3;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

    double k = exp(-M_PI * damping_ratio);
    double a2 = 2. * (1. - EI_SHAPER_VIB_TOL) / (1. + EI_SHAPER_VIB_TOL) * k;
    double a3 = k * k;
    double inv_D = 1. / (1. + a2 + a3);

    is->pulses[0].t = -half_period;
    is->pulses[1].t = 0.;
    is->pulses[2].t = half_period;

    is->pulses[0].a = a3 * inv_D;
    is->pulses[1].a = a2 * inv_D;
    is->pulses[2].a = inv_D;
}

static void
init_shaper_2hump_ei(struct input_shaper *is, double half_period, double damping_ratio)
{
    is->n = 4;
    is->pulses = malloc(is->n * sizeof(struct shaper_pulse));

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

    is->pulses[0].t = -2. * half_period * t4;
    is->pulses[1].t = -2. * half_period * t3;
    is->pulses[2].t = -2. * half_period * t2;
    is->pulses[3].t = -2. * half_period * t1;

    is->pulses[0].a = a4 * inv_D;
    is->pulses[1].a = a3 * inv_D;
    is->pulses[2].a = a2 * inv_D;
    is->pulses[3].a = a1 * inv_D;
}

/****************************************************************
 * Kinematics-related shaper code
 ****************************************************************/

static double
shaper_calc_position(struct stepper_kinematics *sk, struct move *m
                     , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);

    double time = move_time + is->pulses[0].t;
    while (unlikely(time < 0.)) {
        m = list_prev_entry(m, node);
        time += m->move_t;
    }
    double res = 0.;
    for (int i = 0, n = is->n; ; ++i) {
        double pos = is->orig_sk->calc_position_cb(is->orig_sk, m, time);
        res += is->pulses[i].a * pos;
        if (i + 1 >= n)
            break;
        time += is->pulses[i+1].t - is->pulses[i].t;
        while (unlikely(time > m->move_t)) {
            time -= m->move_t;
            m = list_next_entry(m, node);
        }
    }
    return res;
}

static void
shaper_note_generation_time(struct input_shaper *is)
{
    is->sk.gen_steps_pre_active = -is->pulses[0].t;
    is->sk.gen_steps_post_active = is->pulses[is->n-1].t;
}

int __visible
input_shaper_set_sk(struct stepper_kinematics *sk
                    , struct stepper_kinematics *orig_sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    is->sk.calc_position_cb = shaper_calc_position;
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
                               , double damped_spring_period
                               , double damping_ratio
                               , int shaper_type)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);

    if (shaper_type >= ARRAY_SIZE(init_shaper_callbacks) || shaper_type < 0)
        return -1;
    is_init_shaper_callback init_shaper_cb = init_shaper_callbacks[shaper_type];
    free(is->pulses);
    init_shaper_cb(is, .5 * damped_spring_period, damping_ratio);
    shaper_note_generation_time(is);
    return 0;
}

struct stepper_kinematics * __visible
input_shaper_alloc(void)
{
    struct input_shaper *is = malloc(sizeof(*is));
    memset(is, 0, sizeof(*is));
    return &is->sk;
}
