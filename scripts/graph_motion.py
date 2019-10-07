#!/usr/bin/env python2
# Script to graph motion results
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse, datetime
import matplotlib


######################################################################
# Basic trapezoid motion
######################################################################

START_V = 0.
ACCEL = 3000.
CRUISE_V = 100.
ACCEL_T = (CRUISE_V - START_V) / ACCEL
CRUISE_START_D = 0. + ACCEL_T * .5 * (CRUISE_V + START_V)
CRUISE_T = .200
DECEL_START_D = CRUISE_START_D + CRUISE_T * CRUISE_V
END_V = 5.
END_T = (CRUISE_V - END_V) / ACCEL

START_T = .200
TOTAL_T = .600

SEG_TIME = .000100
INV_SEG_TIME = 1. / SEG_TIME

def calc_pos(t):
    if t <= START_T:
        return 0.
    t -= START_T
    if t < ACCEL_T:
        return (START_V + .5 * ACCEL * t) * t
    t -= ACCEL_T
    if t <= CRUISE_T:
        return CRUISE_START_D + CRUISE_V * t
    t -= CRUISE_T
    t = min(t, END_T)
    return DECEL_START_D + (CRUISE_V - .5 * ACCEL * t) * t

def gen_positions(times):
    return [calc_pos(t) for t in times]

def gen_deriv(data):
    return [0.] + [(data[i+1] - data[i]) * INV_SEG_TIME
                   for i in range(len(data)-1)]

def time_to_index(t):
    return int(t * INV_SEG_TIME + .5)


######################################################################
# Pressure advance
######################################################################

PA_SMOOTH_T = .020
PRESSURE_ADVANCE = .045
PRESSURE_ADVANCE_FACTOR = PRESSURE_ADVANCE / (2. * PA_SMOOTH_T)

def calc_pressure_advance_old(t, positions):
    base_pos = positions[time_to_index(t)]
    end_pos = positions[time_to_index(t + PA_SMOOTH_T)]
    return base_pos + (end_pos - base_pos) * 2. * PRESSURE_ADVANCE_FACTOR

def calc_pressure_advance(t, positions):
    base_pos = positions[time_to_index(t)]
    start_pos = positions[time_to_index(t - PA_SMOOTH_T)]
    end_pos = positions[time_to_index(t + PA_SMOOTH_T)]
    return base_pos + (end_pos - start_pos) * PRESSURE_ADVANCE_FACTOR

def calc_pa_integral(t, positions):
    start_index = time_to_index(t - PA_SMOOTH_T) + 1
    mid_index = time_to_index(t)
    end_index = time_to_index(t + PA_SMOOTH_T)
    prev_i = sum(positions[start_index:mid_index]) / (mid_index - start_index)
    post_i = sum(positions[mid_index+1:end_index]) / (end_index - mid_index - 1)
    base_pos = .5 * (prev_i + post_i)
    return base_pos + (post_i - prev_i) * 2. * PRESSURE_ADVANCE_FACTOR

def calc_pa_integral2(t, positions):
    start_index = time_to_index(t - PA_SMOOTH_T) + 1
    end_index = time_to_index(t + PA_SMOOTH_T)
    base_pos = sum(positions[start_index:end_index]) / (end_index - start_index)
    start_pos = positions[time_to_index(t - PA_SMOOTH_T)]
    end_pos = positions[time_to_index(t + PA_SMOOTH_T)]
    return base_pos + (end_pos - start_pos) * PRESSURE_ADVANCE_FACTOR


######################################################################
# Belt spring motion
######################################################################

HALF_SMOOTH_T = .019 * .5
#ADVANCE = .001000
#SPRING_FACTOR = ADVANCE / (2. * SMOOTH_T * SMOOTH_T)
SPRING_FACTOR = 1. / 3.

def calc_position_integral(t, positions):
    start_index = time_to_index(t - HALF_SMOOTH_T) + 1
    end_index = time_to_index(t + HALF_SMOOTH_T)
    return sum(positions[start_index:end_index]) / (end_index - start_index)

def calc_position_smooth(t, positions):
    start_pos = positions[time_to_index(t - HALF_SMOOTH_T)]
    end_pos = positions[time_to_index(t + HALF_SMOOTH_T)]
    return .5 * (start_pos + end_pos)

def calc_belt_spring(t, positions):
    base_pos = positions[time_to_index(t)]
    start_pos = positions[time_to_index(t - HALF_SMOOTH_T)]
    start_diff = base_pos - start_pos
    end_pos = positions[time_to_index(t + HALF_SMOOTH_T)]
    end_diff = end_pos - base_pos
    return base_pos + (end_diff - start_diff) * SPRING_FACTOR

def calc_belt_spring2(t, positions):
    p0 = positions[time_to_index(t - 1.5*HALF_SMOOTH_T)]
    p1 = positions[time_to_index(t - .5*HALF_SMOOTH_T)]
    p2 = positions[time_to_index(t + .5*HALF_SMOOTH_T)]
    p3 = positions[time_to_index(t + 1.5*HALF_SMOOTH_T)]
    return .125 * (p0 + p3) + 3. * .125 * (p1 + p2)


######################################################################
# Plotting and startup
######################################################################

#gen_updated_position = calc_pressure_advance_old
gen_updated_position = calc_pressure_advance
#gen_updated_position = calc_pa_integral2
#gen_updated_position = calc_position_integral
#gen_updated_position = calc_position_smooth
#gen_updated_position = calc_belt_spring
#gen_updated_position = calc_belt_spring2

def plot_motion():
    # Nominal motion
    times = [SEG_TIME * t for t in range(int(TOTAL_T * INV_SEG_TIME))]
    positions = gen_positions(times)
    velocities = gen_deriv(positions)
    accels = gen_deriv(velocities)
    # Updated motion
    drop = int(.100 * INV_SEG_TIME)
    upd_times = times[drop:-drop]
    upd_positions = [gen_updated_position(t, positions) for t in upd_times]
    upd_velocities = gen_deriv(upd_positions)
    upd_accels = gen_deriv(upd_velocities)
    upd_offset = [up - p for p, up in zip(positions[drop:-drop], upd_positions)]
    # Build plot
    fig, (ax1, ax2, ax3) = matplotlib.pyplot.subplots(nrows=3, sharex=True)
    ax1.set_title("Motion")
    ax1.set_ylabel('Velocity (mm/s)')
    ax1.plot(upd_times, upd_velocities, 'r', label='New Velocity', alpha=0.8)
    ax1.plot(times, velocities, 'g', label='Nominal Velocity', alpha=0.8)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.grid(True)
    ax2.set_ylabel('Acceleration (mm/s^2)')
    ax2.plot(upd_times, upd_accels, 'r', label='New Accel', alpha=0.8)
    ax2.plot(times, accels, 'g', label='Nominal Accel', alpha=0.8)
    ax2.set_ylim([-3. * ACCEL, 3. * ACCEL])
    ax2.legend(loc='best', prop=fontP)
    ax2.grid(True)
    ax3.set_ylabel('Offset (mm)')
    ax3.plot(upd_times, upd_offset, 'r', label='Offset', alpha=0.8)
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
