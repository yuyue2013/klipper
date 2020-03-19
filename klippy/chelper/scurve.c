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
    // This max value of accel_comp also limits max acceleration; in AO=6 case:
    // max accel = 60 * accel * accel_comp / accel_t^2 <= 10 * a.
    return fmin(accel_comp, accel_t * accel_t * 0.159);
}

static void
scurve_fill_bezier2(struct scurve *s, double start_accel_v
                  , double effective_accel, double accel_offset_t)
{
    s->c2 = .5 * effective_accel;
    s->c1 = start_accel_v + effective_accel * accel_offset_t;
}

// Determine the coefficients for a 4th order bezier position function
static void
scurve_fill_bezier4(struct scurve *s, double start_accel_v
        , double effective_accel, double total_accel_t, double accel_offset_t
        , double accel_comp)
{
    if (!total_accel_t)
        return;
    double inv_accel_t = 1. / total_accel_t;
    double accel_div_accel_t = effective_accel * inv_accel_t;
    double accel_div_accel_t2 = accel_div_accel_t * inv_accel_t;
    s->c4 = -.5 * accel_div_accel_t2;
    s->c3 = accel_div_accel_t;
    s->c2 = -6. * accel_div_accel_t2 * accel_comp;
    s->c1 = start_accel_v + 6. * accel_div_accel_t * accel_comp;

    s->c1 += ((4. * s->c4 * accel_offset_t
                + 3. * s->c3) * accel_offset_t + 2. * s->c2) * accel_offset_t;
    s->c2 += (6. * s->c4 * accel_offset_t + 3. * s->c3) * accel_offset_t;
    s->c3 += 4. * s->c4 * accel_offset_t;
}

// Determine the coefficients for a 6th order bezier position function
static void
scurve_fill_bezier6(struct scurve *s, double start_accel_v
        , double effective_accel, double total_accel_t, double accel_offset_t
        , double accel_comp)
{
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

    s->c1 += ((((6. * s->c6 * accel_offset_t + 5. * s->c5) * accel_offset_t
                    + 4. * s->c4) * accel_offset_t
                + 3. * s->c3) * accel_offset_t + 2. * s->c2) * accel_offset_t;
    s->c2 += (((15. * s->c6 * accel_offset_t + 10. * s->c5) * accel_offset_t
                + 6. * s->c4) * accel_offset_t + 3. * s->c3) * accel_offset_t;
    s->c3 += ((20. * s->c6 * accel_offset_t
                + 10. * s->c5) * accel_offset_t + 4. * s->c4) * accel_offset_t;
    s->c4 += (15. * s->c6 * accel_offset_t + 5. * s->c5) * accel_offset_t;
    s->c5 += 6. * s->c6 * accel_offset_t;
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

// Compute Bezier curve difference between end and start: s(end) - s(start)
double
scurve_diff(struct scurve *s, double start, double end)
{
    double mid = (start + end) * 0.5;
    double h = end - start;
    double h2 = h * h;

    double a6 = s->c6;
    double a5 = s->c5;
    double a4 = s->c4;
    double a3 = s->c3;
    double a2 = s->c2;
    double a1 = s->c1;

    double v = 6. * a6;
    v = 5. * a5 + v * mid;
    a6 *= h2;
    v = 4. * a4 + 5. * a6 + v * mid;
    a5 *= h2;
    v = 3. * a3 + (5./2.) * a5 + v * mid;
    v = 2. * a2 + h2 * (a4 + (3./8.) * a6) + v * mid;
    v = a1 + h2 * (0.25 * a3 + (1./16.) * a5) + v * mid;
    return v * h;
}

// Compute integral of Bezier curve derivative times t:
// integral(s'(t) * t, t = start..end)
double
scurve_deriv_t_integrate(struct scurve *s, double start, double end)
{
    double mid = (start + end) * 0.5;
    double h = end - start;
    double h2 = h * h;

    double a6 = s->c6;
    double a5 = s->c5;
    double a4 = s->c4;
    double a3 = s->c3;
    double a2 = s->c2;
    double a1 = s->c1;

    double v = 6. * a6;
    v = 5. * a5 + v * mid;
    a6 *= h2;
    v = 4. * a4 + 7.5 * a6 + v * mid;
    a5 *= h2;
    v = 3. * a3 + (25./6.) * a5 + v * mid;
    a4 *= h2;
    a6 *= h2;
    v = 2. * a2 + 2. * a4 + (9./8.) * a6 + v * mid;
    v = a1 + h2 * (0.75 * a3 + (5./16.) * a5) + v * mid;
    v = h2 * ((1./6.) * a2 + (1./20.) * a4 + (3./224.) * a6) + v * mid;
    return v * h;
}

// Integrate Bezier curve over (start; end) interval:
// integral(s(t), t = start..end)
double
scurve_integrate(struct scurve *s, double start, double end)
{
    double mid = (start + end) * 0.5;
    double h = end - start;
    double h2 = h * h;

    double a6 = s->c6;
    double a5 = s->c5;
    double a4 = s->c4;
    double a3 = s->c3;
    double a2 = s->c2;
    double a1 = s->c1;

    double v = a6;
    v = a5 + v * mid;
    a6 *= h2;
    v = a4 + (5./4.) * a6 + v * mid;
    a5 *= h2;
    v = a3 + (5./6.) * a5 + v * mid;
    a4 *= h2;
    a6 *= h2;
    v = a2 + 0.5 * a4 + (3./16.) * a6 + v * mid;
    v = a1 + h2 * (0.25 * a3 + (1./16.) * a5) + v * mid;
    v = h2 * ((1./12.) * a2 + (1./80.) * a4 + (1./448.) * a6) + v * mid;
    return v * h;
}

// Integrate Bezier curve times t over (start; end) interval:
// integral(s(t) * t, t = start..end)
double
scurve_integrate_t(struct scurve *s, double start, double end)
{
    double mid = (start + end) * 0.5;
    double h = end - start;
    double h2 = h * h;

    double a6 = s->c6;
    double a5 = s->c5;
    double a4 = s->c4;
    double a3 = s->c3;
    double a2 = s->c2;
    double a1 = s->c1;

    double v = a6;
    v = a5 + v * mid;
    a6 *= h2;
    v = a4 + (7./4.) * a6 + v * mid;
    a5 *= h2;
    v = a3 + (5./4.) * a5 + v * mid;
    a4 *= h2;
    a6 *= h2;
    v = a2 + (5./6.) * a4 + (7./16.) * a6 + v * mid;
    a3 *= h2;
    a5 *= h2;
    v = a1 + 0.5 * a3 + (3./16.) * a5 + v * mid;
    v = h2 * (0.25 * a2 + (1./16.) * a4 + (1./64.) * a6) + v * mid;
    v = h2 * ((1./12.) * a1 + (1./80.) * a3 + (1./448.) * a5) + v * mid;
    return v * h;
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
