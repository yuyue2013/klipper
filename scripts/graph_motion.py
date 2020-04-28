#!/usr/bin/env python2
# Script to graph motion results
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse, datetime, math
import matplotlib

SEG_TIME = .000100
INV_SEG_TIME = 1. / SEG_TIME

SPRING_FREQ=35.0
DAMPING_RATIO=0.05

MEASURED_FREQ=40.0
MEAS_DAMPING_RATIO=0.003

######################################################################
# Basic trapezoid motion
######################################################################

# List of moves: [(start_v, end_v, move_t), ...]
Moves = [
    # X velocities from: 0,0 -> 0,20 -> 40,40 -> 80,40 -> 80,80
    (0., 0., .100),
    (6.869, 89.443, None), (89.443, 89.443, .200), (89.443, 17.361, None),
    (19.410, 100., None), (100., 100., .200), (100., 5., None),
    (0., 0., .300)
]
ACCEL = 3000.
MAX_JERK = ACCEL * 0.6 * SPRING_FREQ

def get_accel(start_v, end_v):
    return ACCEL

def get_accel_jerk_limit(start_v, end_v):
    effective_accel = math.sqrt(MAX_JERK * abs(end_v - start_v) / 6.)
    return min(effective_accel, ACCEL)

# Standard constant acceleration generator
def get_acc_pos_ao2(rel_t, start_v, accel, move_t):
    return (start_v + 0.5 * accel * rel_t) * rel_t

# Bezier curve "accel_order=4" generator
def get_acc_pos_ao4(rel_t, start_v, accel, move_t):
    inv_accel_t = 1. / move_t
    accel_div_accel_t = accel * inv_accel_t
    accel_div_accel_t2 = accel_div_accel_t * inv_accel_t

    c4 = -.5 * accel_div_accel_t2;
    c3 = accel_div_accel_t;
    c1 = start_v
    return ((c4 * rel_t + c3) * rel_t * rel_t + c1) * rel_t

# Bezier curve "accel_order=6" generator
def get_acc_pos_ao6(rel_t, start_v, accel, move_t):
    inv_accel_t = 1. / move_t
    accel_div_accel_t = accel * inv_accel_t
    accel_div_accel_t2 = accel_div_accel_t * inv_accel_t
    accel_div_accel_t3 = accel_div_accel_t2 * inv_accel_t
    accel_div_accel_t4 = accel_div_accel_t3 * inv_accel_t

    c6 = accel_div_accel_t4;
    c5 = -3. * accel_div_accel_t3;
    c4 = 2.5 * accel_div_accel_t2;
    c1 = start_v;
    return (((c6 * rel_t + c5) * rel_t + c4)
            * rel_t * rel_t * rel_t + c1) * rel_t

get_acc_pos = get_acc_pos_ao2
get_acc = get_accel

# Calculate positions based on 'Moves' list
def gen_positions():
    out = []
    start_d = start_t = t = 0.
    for start_v, end_v, move_t in Moves:
        if move_t is None:
            move_t = abs(end_v - start_v) / get_acc(start_v, end_v)
        accel = (end_v - start_v) / move_t
        end_t = start_t + move_t
        while t <= end_t:
            rel_t = t - start_t
            out.append(start_d + get_acc_pos(rel_t, start_v, accel, move_t))
            t += SEG_TIME
        start_d += get_acc_pos(move_t, start_v, accel, move_t)
        start_t = end_t
    return out


######################################################################
# Estimated motion with belt as spring
######################################################################

def estimate_spring(positions):
    ang_freq2 = (SPRING_FREQ * 2. * math.pi)**2
    damping_factor = 4. * math.pi * DAMPING_RATIO * SPRING_FREQ
    head_pos = head_v = 0.
    out = []
    for stepper_pos in positions:
        head_pos += head_v * SEG_TIME
        head_a = (stepper_pos - head_pos) * ang_freq2
        head_v += head_a * SEG_TIME
        head_v -= head_v * damping_factor * SEG_TIME
        out.append(head_pos)
    return out


######################################################################
# List helper functions
######################################################################

MARGIN_TIME = 0.050

def time_to_index(t):
    return int(t * INV_SEG_TIME + .5)

def indexes(positions):
    drop = time_to_index(MARGIN_TIME)
    return range(drop, len(positions)-drop)

def trim_lists(*lists):
    keep = len(lists[0]) - time_to_index(2. * MARGIN_TIME)
    for l in lists:
        del l[keep:]


######################################################################
# Common data filters
######################################################################

# Generate estimated first order derivative
def gen_deriv(data):
    return [0.] + [(data[i+1] - data[i]) * INV_SEG_TIME
                   for i in range(len(data)-1)]

# Simple average between two points smooth_time away
def calc_average(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = .5 * (positions[i-offset] + positions[i+offset])
    return out

# Average (via integration) of smooth_time range
def calc_smooth(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / (2*offset - 1)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = sum(positions[i-offset+1:i+offset]) * weight
    return out

# Time weighted average (via integration) of smooth_time range
def calc_weighted(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / offset**2
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset - abs(j-i))
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`h**2 - (t-T)**2`) of smooth_time range
def calc_weighted2(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = .75 / offset**3
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset**2 - (j-i)**2)
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`(h**2 - (t-T)**2)**2`) of smooth_time range
def calc_weighted4(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 15 / (16. * offset**5)
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * ((offset**2 - (j-i)**2))**2
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`(h - abs(t-T))**2 * (2 * abs(t-T) + h)`) of range
def calc_weighted3(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / offset**4
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset - abs(j-i))**2
                         * (2. * abs(j-i) + offset)
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out


######################################################################
# Spring motion estimation
######################################################################

SPRING_ADVANCE = .000020
RESISTANCE_ADVANCE = 0.

def calc_spring_raw(positions):
    sa = SPRING_ADVANCE * INV_SEG_TIME * INV_SEG_TIME
    ra = RESISTANCE_ADVANCE * INV_SEG_TIME
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i]
                  + sa * (positions[i-1] - 2.*positions[i] + positions[i+1])
                  + ra * (positions[i+1] - positions[i]))
    return out

def calc_spring_double_weighted(positions, smooth_time):
    offset = time_to_index(smooth_time * .25)
    sa = SPRING_ADVANCE * (INV_SEG_TIME / offset)**2
    ra = RESISTANCE_ADVANCE * INV_SEG_TIME
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i]
                  + sa * (positions[i-offset] - 2.*positions[i]
                          + positions[i+offset])
                  + ra * (positions[i+1] - positions[i]))
    return calc_weighted(out, smooth_time=.5 * smooth_time)

def calc_zv_shaper(positions):
    df = math.sqrt(1. - MEAS_DAMPING_RATIO**2)
    K = math.exp(-MEAS_DAMPING_RATIO * math.pi / df)
    inv_D = 1. / (1. + K)
    t_d = 1. / (MEASURED_FREQ * df)
    offset = time_to_index(t_d * .25)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i + offset] + positions[i - offset] * K) * inv_D
    return out

def calc_zvd_shaper(positions):
    df = math.sqrt(1. - MEAS_DAMPING_RATIO**2)
    K = math.exp(-MEAS_DAMPING_RATIO * math.pi / df)
    inv_D = 1. / (1. + 2.*K + K**2)
    t_d = 1. / (MEASURED_FREQ * df)
    offset = time_to_index(t_d * .5)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i + offset] + positions[i] * 2.*K
                + positions[i - offset] * K**2) * inv_D
    return out

def calc_zvdd_shaper(positions):
    df = math.sqrt(1. - MEAS_DAMPING_RATIO**2)
    K = math.exp(-MEAS_DAMPING_RATIO * math.pi / df)
    inv_D = 1. / (1. + 3.*K + 3.*K**2 + K**3)
    t_d = 1. / (MEASURED_FREQ * df)
    offset1 = time_to_index(t_d * .25)
    offset2 = time_to_index(t_d * .75)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = inv_D * (positions[i + offset2] + positions[i + offset1] * 3.*K
                + positions[i - offset1] * 3.*K**2 + positions[i - offset2] * K**3)
    return out

def calc_ei_shaper(positions):
    v_tol = 0.05 # vibration tolerance
    d_r = MEAS_DAMPING_RATIO
    td = 1. / (MEASURED_FREQ * math.sqrt(1. - d_r**2))

    a2 = 2. * (1. - v_tol) / (1. + v_tol) * math.exp(-math.pi * d_r)
    a3 = math.exp(-2. * math.pi * d_r)
    inv_D = 1. / (1. + a2 + a3)

    offset1 = time_to_index(.5 * td)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i + offset1]
            + a2 * positions[i]
            + a3 * positions[i - offset1]) * inv_D
    return out

def calc_2hump_ei_shaper(positions):
    d_r = MEAS_DAMPING_RATIO
    td = 1. / (MEASURED_FREQ * math.sqrt(1. - d_r**2))
    # Coefficients calculated for 5% vibration tolerance
    t2 = 0.49890 + 0.16270 * d_r - 0.54262 * d_r**2 + 6.16180 * d_r**3
    t3 = 0.99748 + 0.18382 * d_r - 1.58270 * d_r**2 + 8.17120 * d_r**3
    t4 = 1.49920 - 0.09297 * d_r - 0.28338 * d_r**2 + 1.85710 * d_r**3

    a1 = 0.16054 + 0.76699 * d_r + 2.26560 * d_r**2 - 1.22750 * d_r**3
    a2 = 0.33911 + 0.45081 * d_r - 2.58080 * d_r**2 + 1.73650 * d_r**3
    a3 = 0.34089 - 0.61533 * d_r - 0.68765 * d_r**2 + 0.42261 * d_r**3
    a4 = 0.15997 - 0.60246 * d_r + 1.00280 * d_r**2 - 0.93145 * d_r**3

    offset1 = time_to_index(-.75 * td)
    offset2 = time_to_index((t2 - .75) * td)
    offset3 = time_to_index((t3 - .75) * td)
    offset4 = time_to_index((t4 - .75) * td)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (a1 * positions[i - offset1]
            + a2 * positions[i - offset2]
            + a3 * positions[i - offset3]
            + a4 * positions[i - offset4])
    return out

# Ideal values
SPRING_ADVANCE = 1. / ((MEASURED_FREQ * 2. * math.pi)**2)
RESISTANCE_ADVANCE = 2. * MEAS_DAMPING_RATIO * math.sqrt(SPRING_ADVANCE)
SMOOTH_TIME = (2./3.) * 2. * math.pi * math.sqrt(SPRING_ADVANCE)

def gen_updated_position(positions):
    #return calc_weighted(positions, 0.040)
    #return calc_spring_double_weighted(positions, SMOOTH_TIME)
    #return calc_weighted4(calc_spring_raw(positions), SMOOTH_TIME)
    return calc_ei_shaper(positions)


######################################################################
# Plotting and startup
######################################################################

def plot_motion():
    # Nominal motion
    positions = gen_positions()
    velocities = gen_deriv(positions)
    accels = gen_deriv(velocities)
    # Updated motion
    upd_positions = gen_updated_position(positions)
    upd_velocities = gen_deriv(upd_positions)
    upd_accels = gen_deriv(upd_velocities)
    # Estimated position with model of belt as spring
    spring_orig = estimate_spring(positions)
    spring_upd = estimate_spring(upd_positions)
    spring_diff_orig = [n-o for n, o in zip(spring_orig, positions)]
    spring_diff_upd = [n-o for n, o in zip(spring_upd, positions)]
    head_velocities = gen_deriv(spring_orig)
    head_accels = gen_deriv(head_velocities)
    head_upd_velocities = gen_deriv(spring_upd)
    head_upd_accels = gen_deriv(head_upd_velocities)
    # Build plot
    times = [SEG_TIME * i for i in range(len(positions))]
    trim_lists(times, velocities, accels,
               upd_velocities, upd_velocities, upd_accels,
               spring_diff_orig, spring_diff_upd,
               head_velocities, head_upd_velocities,
               head_accels, head_upd_accels)
    fig, (ax1, ax2, ax3) = matplotlib.pyplot.subplots(nrows=3, sharex=True)
    ax1.set_title("Simulation (belt frequency=%.3f, damping_ratio=%.3f)\n"
                  "(measured belt frequency=%.3f, damping_ratio = %.3f)"
                  % (SPRING_FREQ, DAMPING_RATIO, MEASURED_FREQ
                      , MEAS_DAMPING_RATIO))
    ax1.set_ylabel('Velocity (mm/s)')
    ax1.plot(times, upd_velocities, 'r', label='New Velocity', alpha=0.8)
    ax1.plot(times, velocities, 'g', label='Nominal Velocity', alpha=0.8)
    ax1.plot(times, head_velocities, label='Head Velocity', alpha=0.4)
    ax1.plot(times, head_upd_velocities, label='New Head Velocity', alpha=0.4)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.grid(True)
    ax2.set_ylabel('Acceleration (mm/s^2)')
    ax2.plot(times, upd_accels, 'r', label='New Accel', alpha=0.8)
    ax2.plot(times, accels, 'g', label='Nominal Accel', alpha=0.8)
    ax2.plot(times, head_accels, alpha=0.4)
    ax2.plot(times, head_upd_accels, alpha=0.4)
    ax2.set_ylim([-5. * ACCEL, 5. * ACCEL])
    ax2.legend(loc='best', prop=fontP)
    ax2.grid(True)
    ax3.set_ylabel('Deviation (mm)')
    ax3.plot(times, spring_diff_upd, 'r', label='New', alpha=0.8)
    ax3.plot(times, spring_diff_orig, 'g', label='Nominal', alpha=0.8)
    ax3.grid(True)
    ax3.legend(loc='best', prop=fontP)
    ax3.set_xlabel('Time (s)')
    return fig

def setup_matplotlib(output_to_file):
    global matplotlib
    if output_to_file:
        matplotlib.use('Agg')
    import matplotlib.pyplot, matplotlib.dates, matplotlib.font_manager
    import matplotlib.ticker

def main():
    # Parse command-line arguments
    usage = "%prog [options]"
    opts = optparse.OptionParser(usage)
    opts.add_option("-o", "--output", type="string", dest="output",
                    default=None, help="filename of output graph")
    options, args = opts.parse_args()
    if len(args) != 0:
        opts.error("Incorrect number of arguments")

    # Draw graph
    setup_matplotlib(options.output is not None)
    fig = plot_motion()

    # Show graph
    if options.output is None:
        matplotlib.pyplot.show()
    else:
        fig.set_size_inches(8, 6)
        fig.savefig(options.output)

if __name__ == '__main__':
    main()
