#ifndef SCURVE_H
#define SCURVE_H

struct scurve {
    double c1, c2, c3, c4, c5, c6;
};

// Find the distance travel on an S-Curve
static inline double
scurve_eval(struct scurve *s, double time)
{
    double v = s->c6;
    v = s->c5 + v * time;
    v = s->c4 + v * time;
    v = s->c3 + v * time;
    v = s->c2 + v * time;
    v = s->c1 + v * time;
    return v * time;
}

void scurve_fill(struct scurve *s, int accel_order
        , double accel_t, double accel_offset_t, double total_accel_t
        , double start_accel_v, double effective_accel, double accel_comp);
double scurve_get_time(struct scurve *s, double max_scurve_t, double distance);
double scurve_diff(struct scurve *s, double start, double end);
double scurve_deriv_t_integrate(struct scurve *s, double start, double end);
double scurve_integrate(struct scurve *s, double start, double end);
double scurve_integrate_t(struct scurve *s, double start, double end);

#endif // scurve.h
