#ifndef ACCELCOMBINE_H
#define ACCELCOMBINE_H

#include "list.h"

struct accel_group;

struct accel_combiner {
    struct list_head junctions;
    double junct_start_v2;
};

void init_combiner(struct accel_combiner *ac);
void reset_combiner(struct accel_combiner *ac);
void reset_junctions(struct accel_combiner *ac, double start_v2);
void process_next_accel(struct accel_combiner *ac, struct accel_group *ag
                        , double junction_max_v2);

#endif  // accelcombine.h
