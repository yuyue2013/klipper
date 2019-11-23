// Bezier curve acceleration
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // fabs
#include <string.h> // memset

#include "compiler.h" // likely
#include "scurve.h" // scurve

static inline double
max_accel_comp(double accel_comp, double accel_t)
{
    // Limit compensation to maintain velocity > 0 (no movement backwards).
    // 0.159 is a magic number - a solution of optimization problem for AO=6:
    // maximum compensation value such that velocity >= 0 for any accel_t. It is
    // slightly smaller than 1/6 - a solution of the same problem for AO=4.
    return fmin(accel_comp, accel_t * accel_t * 0.159);
}

static void
scurve_fill_bezier2(struct scurve *s, double start_accel_v
                  , double effective_accel, double accel_offset_t)
{
    s->offset_t = accel_offset_t;
    s->c2 = .5 * effective_accel;
    s->c1 = start_accel_v;
    s->c0 = (-s->c2 * s->offset_t - s->c1) * s->offset_t;
}

// Determine the coefficients for a 4th order bezier position function
static void
scurve_fill_bezier4(struct scurve *s, double start_accel_v
        , double effective_accel, double total_accel_t, double accel_offset_t
        , double accel_comp)
{
    s->offset_t = accel_offset_t;
    if (!total_accel_t)
        return;
    double inv_accel_t = 1. / total_accel_t;
    double accel_div_accel_t = effective_accel * inv_accel_t;
    double accel_div_accel_t2 = accel_div_accel_t * inv_accel_t;
    s->c4 = -.5 * accel_div_accel_t2;
    s->c3 = accel_div_accel_t;
    s->c2 = -6. * accel_div_accel_t2 * accel_comp;
    s->c1 = start_accel_v + 6. * accel_div_accel_t * accel_comp;
    s->c0 = -scurve_eval(s, 0);
}

// Determine the coefficients for a 6th order bezier position function
static void
scurve_fill_bezier6(struct scurve *s, double start_accel_v
        , double effective_accel, double total_accel_t, double accel_offset_t
        , double accel_comp)
{
    s->offset_t = accel_offset_t;
    if (!total_accel_t)
        return;
    double inv_accel_t = 1. / total_accel_t;
    double accel_div_accel_t2 = effective_accel * inv_accel_t * inv_accel_t;
    double accel_div_accel_t3 = accel_div_accel_t2 * inv_accel_t;
    double accel_div_accel_t4 = accel_div_accel_t3 * inv_accel_t;
    s->c6 = accel_div_accel_t4;
    s->c5 = -3. * accel_div_accel_t3;
    s->c4 = 2.5 * accel_div_accel_t2 + 30. * accel_div_accel_t4 * accel_comp;
    s->c3 = -60. * accel_div_accel_t3 * accel_comp;
    s->c2 = 30. * accel_div_accel_t2 * accel_comp;
    s->c1 = start_accel_v;
    s->c0 = -scurve_eval(s, 0);
}

double scurve_get_time(struct scurve *s, double max_scurve_t, double distance)
{
    double low = 0;
    double high = max_scurve_t;
    if (scurve_eval(s, high) <= distance) return high;
    if (scurve_eval(s, low) > distance) return low;
    while (likely(high - low > .000000001)) {
        double guess_time = (high + low) * .5;
        if (scurve_eval(s, guess_time) > distance) {
            high = guess_time;
        } else {
            low = guess_time;
        }
    }
    return (high + low) * .5;
}

static inline double
scurve_antiderivative(struct scurve *s, double time)
{
    time += s->offset_t;
    double v = (1./7.) * s->c6;
    v = (1./6.) * s->c5 + v * time;
    v = (1./5.) * s->c4 + v * time;
    v = (1./4.) * s->c3 + v * time;
    v = (1./3.) * s->c2 + v * time;
    v = (1./2.) * s->c1 + v * time;
    v = s->c0 + v * time;
    return v * time;
}

double
scurve_integrate(struct scurve *s, double start, double end)
{
    return scurve_antiderivative(s, end) - scurve_antiderivative(s, start);
}

void
scurve_fill(struct scurve *s, int accel_order
            , double accel_t, double accel_offset_t, double total_accel_t
            , double start_accel_v, double effective_accel, double accel_comp)
{
    memset(s, 0, sizeof(*s));
    if (accel_order == 4) {
        scurve_fill_bezier4(s, start_accel_v, effective_accel
                , total_accel_t, accel_offset_t
                , max_accel_comp(accel_comp, total_accel_t));
    } else if (accel_order == 6) {
        scurve_fill_bezier6(s, start_accel_v, effective_accel
                , total_accel_t, accel_offset_t
                , max_accel_comp(accel_comp, total_accel_t));
    } else {
        scurve_fill_bezier2(s, start_accel_v, effective_accel, accel_offset_t);
    }
}
