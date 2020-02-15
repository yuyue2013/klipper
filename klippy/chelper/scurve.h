#ifndef SCURVE_H
#define SCURVE_H

struct scurve {
    double c1, c2, c3, c4, c5, c6;
    double total_accel_t;
};

// Find the distance travel on an S-Curve
double scurve_eval(const struct scurve *s, double time);

void scurve_offset(struct scurve *s, double offset_t);
void scurve_fill(struct scurve *s, int accel_order
        , double accel_t, double accel_offset_t, double total_accel_t
        , double start_accel_v, double effective_accel);
double scurve_get_time(const struct scurve *s, double distance);
double scurve_diff(struct scurve *s, double start, double end);
double scurve_deriv_t_integrate(struct scurve *s, double start, double end);
double scurve_integrate(struct scurve *s, double start, double end);
double scurve_integrate_t(struct scurve *s, double start, double end);

#endif // scurve.h
