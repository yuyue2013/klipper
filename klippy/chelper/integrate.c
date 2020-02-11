// Helpers to integrate the smoothing weight function.
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "integrate.h"

// Calculate (-3*t^6 + 7*h^2*t^4 - 5*h^4*t^2 + h^6)
static inline double
w(double h, double t)
{
    double t2 = t*t;
    double h2 = h*h;
    double h4 = h2*h2;
    return ((-3. * t2 + 7. * h2) * t2 - 5. * h4) * t2 + h4 * h2;
}

// Integrate (-3*t^6 + 7*h^2*t^4 - 5*h^4*t^2 + h^6)
static inline double
iwt0(double h, double t)
{
    double t2 = t*t;
    double h2 = h*h;
    double h4 = h2*h2;
    return ((((-3./7.) * t2 + (7./5.) * h2) * t2 - (5./3.) * h4) * t2
            + h4 * h2) * t;
}

// Integrate t * (-3*t^6 + 7*h^2*t^4 - 5*h^4*t^2 + h^6)
static inline double
iwt1(double h, double t)
{
    double t2 = t*t;
    double h2 = h*h;
    double h4 = h2*h2;
    return ((((-3./8.) * t2 + (7./6.) * h2) * t2 - (5./4.) * h4) * t2
            + .5 * h4 * h2) * t2;
}

// Integrate t^2 * (-3*t^6 + 7*h^2*t^4 - 5*h^4*t^2 + h^6)
static inline double
iwt2(double h, double t)
{
    double t2 = t*t;
    double h2 = h*h;
    double h4 = h2*h2;
    return ((((-1./3.) * t2 + h2) * t2 - h4) * t2 + (1./3.) * h4 * h2) * t2 * t;
}

// Integrate (pos + start_v*t + half_accel*t^2) with smoothing weight function
// over the range [start; end] with T == -toff
double
integrate_weighted(double pos, double start_v, double half_accel
                   , double start, double end, double toff, double hst)
{
    // Substitute the integration variable tnew = t + toff to simplify integrals
    pos += (half_accel * toff - start_v) * toff;
    start_v -= 2. * half_accel * toff;
    start += toff; end += toff;
    double res = half_accel * (iwt2(hst, end) - iwt2(hst, start));
    res += start_v * (iwt1(hst, end) - iwt1(hst, start));
    res += pos * (iwt0(hst, end) - iwt0(hst, start));
    return res;
}

// Integrate velocity jumps near the ends of the range [start; end] with
// smoothing weight function. To get correct results it is required to sum up
// the returned values over the full integration range [T-hst; T+hst].
double
integrate_velocity_jumps(double start_v, double half_accel
                         , double start, double end, double toff, double hst)
{
    double end_v = start_v + 2. * half_accel * end;
    start_v += 2. * half_accel * start;
    // Velocity jumps integration assumes that the weight function vanishes at
    // the integration bounds T-hst and T+hst to ignore velocity jumps there.
    return start_v * w(hst, start + toff) - end_v * w(hst, end + toff);
}

// Calculate the inverse of the norm of the weight function
double
calc_inv_norm(double hst)
{
    // Inverse norm of weight function (-3*t^6 + 7*h^2*t^4 - 5*h^4*t^2 + h^6)
    return 105. / (64. * hst * hst * hst * hst * hst * hst * hst);
}
