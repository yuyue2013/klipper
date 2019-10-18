# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import copy, math, logging, importlib
import mcu, homing, chelper, kinematics.extruder

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

class error(Exception):
    pass

EPSILON = 0.000000001

class Acceleration:
    def __init__(self, move, accel, jerk=None):
        toolhead = move.toolhead
        self.ffi_lib = toolhead.ffi_lib
        self.move = move
        self.start_accel = self
        self.prev_accel = None
        self.max_accel = accel
        self.accel_order = toolhead.accel_order
        self.jerk = jerk or move.max_jerk
        self.min_jerk_limit_time = toolhead.min_jerk_limit_time
        self.min_accel = min(self.max_accel
                , self.jerk * self.min_jerk_limit_time / 6.)
        self.max_start_v2 = 0.
        self.min_start_time = 0.
        self.combined_d = move.move_d
        self.accel_d = 0.
        self.accel_t = self.accel_offset_t = self.total_accel_t = 0.
        self.start_accel_v = 0.
        self.effective_accel = 0.
    def limit_accel(self, accel, jerk=None):
        self.max_accel = min(self.max_accel, accel)
        if jerk and jerk < self.jerk:
            self.jerk = jerk
        self.min_accel = min(self.max_accel
                , self.jerk * self.min_jerk_limit_time / 6.)
    def set_max_start_v2(self, max_start_v2):
        self.max_start_v2 = max_start_v2
        self.max_start_v = math.sqrt(max_start_v2)
    def calc_max_v2(self, delta_d=0.):
        dist = self.combined_d + delta_d
        start_v2 = self.start_accel.max_start_v2
        start_v = self.start_accel.max_start_v
        # Check if accel is the limiting factor
        max_accel_v2 = start_v2 + 2.0 * dist * self.max_accel
        if self.accel_order == 2:
            return max_accel_v2
        # Compute maximum achievable speed with limited kinematic jerk using
        # max(jerk) == 6 * accel / accel_time, which is exact for accel order 4
        # and is quite accurate for accel order 6:
        # max(jerk) == 10 / sqrt(3) * accel / accel_time ~=
        #     5.774 * accel / accel_time
        # This leads to the cubic equation
        # (max_v^2 - start_v^2) * (max_v + start_v) / 2 ==
        #     dist^2 * jerk / 3
        # which is solved using Cardano's formula.
        a = 2./3. * start_v
        b = a*a*a
        c = dist * dist * self.jerk / 3.
        if b * 54 < c:
            # Make max_v monotonic over start_v: return the max velocity
            # which works for any start_v velocity below the threshold.
            # Combine algorithm relies on monotonicity of max_v(start_v).
            max_v = 1.5 * (c*.5)**(1./3.)
        else:
            d = math.sqrt(c * (c + 2 * b))
            e = (b + c + d)**(1./3.)
            if e < EPSILON:
                return start_v
            max_v = e + a*a / e - start_v / 3.
        return max(min(max_v * max_v, max_accel_v2)
                , start_v2 + 2.0 * dist * self.min_accel)
    def calc_effective_accel(self, start_v, cruise_v):
        if self.accel_order == 2:
            return self.max_accel
        effective_accel = max(self.min_accel, min(self.max_accel
            , math.sqrt(self.jerk * (cruise_v - start_v) / 6.)))
        return effective_accel
    def calc_min_accel_time(self, start_v, cruise_v):
        delta_v = cruise_v - start_v
        min_accel_time = delta_v / self.max_accel
        if self.accel_order > 2:
            min_accel_time = max(min_accel_time
                    , math.sqrt(6. * delta_v / self.jerk))
        if self.min_accel:
            min_accel_time = min(min_accel_time, delta_v / self.min_accel)
        return min_accel_time
    def calc_min_accel_dist(self, cruise_v):
        start_v = self.start_accel.max_start_v
        if cruise_v <= start_v: return 0.
        accel_t = self.calc_min_accel_time(start_v, cruise_v)
        return (start_v + cruise_v) * 0.5 * accel_t
    def move_get_time(self, cmove, accel_d):
        if accel_d < EPSILON: return 0.
        return self.ffi_lib.move_get_time(cmove, accel_d)
    def get_accel_list(self):
        accel_list = []
        a = self
        while True:
            accel_list.append(a)
            if a == self.start_accel: break
            a = a.prev_accel
        return reversed(accel_list)
    def set_junction(self, cruise_v2, time_offset_from_start=True):
        combined = self
        if combined.start_accel.max_start_v2 > cruise_v2:
            combined.start_accel.set_max_start_v2(cruise_v2)
        start_accel_v2 = combined.start_accel.max_start_v2
        start_accel_v = combined.start_accel.max_start_v
        cruise_v = math.sqrt(cruise_v2)
        avg_v = (cruise_v + start_accel_v) * 0.5
        combined_accel_t = combined.calc_min_accel_time(start_accel_v, cruise_v)
        combined_accel_d = avg_v * combined_accel_t
        if combined_accel_d > combined.combined_d + EPSILON:
            raise error(
                    'Logic error: need %.6lf to accelerate, only %.6lf combined'
                    % (combined_accel_d, combined.combined_d))
        effective_accel = combined.calc_effective_accel(
                start_accel_v, cruise_v)
        cmove = combined.move.cmove
        toolhead = self.move.toolhead
        toolhead.move_fill(
            cmove, 0.,
            combined_accel_t, 0., combined_accel_t,
            0.,
            0., 0., 0.,
            0., 0., 0.,
            0., 1., 0.,
            start_accel_v, cruise_v, effective_accel, 0.,
            toolhead.accel_compensation)
        remaining_accel_t = combined_accel_t
        remaining_accel_d = combined_accel_d
        accel_list = self.get_accel_list()
        for a in accel_list:
            a.move.cruise_v = cruise_v
            if remaining_accel_d <= 0: continue
            a.effective_accel = effective_accel
            a.total_accel_t = combined_accel_t
            a.accel_d = min(a.move.move_d, remaining_accel_d)
            a.start_accel_v = start_accel_v
            if time_offset_from_start:
                a.accel_offset_t = combined_accel_t - remaining_accel_t
                a.accel_t = combined.move_get_time(cmove, a.accel_d +
                        combined_accel_d - remaining_accel_d) - a.accel_offset_t
            else:
                a.accel_offset_t = combined_accel_t - combined.move_get_time(
                        cmove, combined_accel_d - remaining_accel_d + a.accel_d)
                a.accel_t = remaining_accel_t - a.accel_offset_t
            remaining_accel_t -= a.accel_t
            remaining_accel_d -= a.move.move_d

class AccelCombiner:
    def __init__(self, max_start_v2):
        self.reset(max_start_v2)
    def reset(self, max_start_v2):
        self.references = []
        self.max_start_v2 = max_start_v2
    def calc_min_accel_end_time(self, accel, cruise_v2):
        if accel.start_accel.max_start_v2 >= cruise_v2:
            return accel.combined_d / cruise_v2 + accel.min_start_time
        start_v = accel.start_accel.max_start_v
        cruise_v = math.sqrt(cruise_v2)
        accel_t = accel.calc_min_accel_time(start_v, cruise_v)
        cruise_t = (accel.combined_d
                - (start_v + cruise_v) * 0.5 * accel_t) / cruise_v
        return accel_t + cruise_t + accel.min_start_time
    def process_next_accel(self, accel, junction_max_v2):
        refs = self.references
        prev_accel = refs[-1][1] if refs else None
        accel.prev_accel = prev_accel
        max_cruise_v2 = accel.move.max_cruise_v2
        accel.set_max_start_v2(min(junction_max_v2, max_cruise_v2
            , min(prev_accel.max_end_v2, prev_accel.move.max_cruise_v2)
            if prev_accel else self.max_start_v2))
        if prev_accel:
            accel.min_start_time = prev_accel.min_end_time
        accel.max_end_v2 = accel.calc_max_v2()
        accel.min_end_time = self.calc_min_accel_end_time(accel
                , min(accel.max_end_v2, max_cruise_v2))
        if (not prev_accel or prev_accel.accel_order != accel.accel_order
                or accel.accel_order == 2):
            del refs[:]
        junction_accel_limit_v2 = junction_max_v2 * (53. / 54.)
        while refs and refs[-1][0].max_start_v2 >= min(accel.max_start_v2
                , junction_accel_limit_v2):
            refs.pop()
        refs.append((copy.copy(accel), accel))
        refs[-1][0].start_accel = refs[-1][0]
        for i in range(len(refs)-2, -1, -1):
            a_ref, a_m = refs[i]
            # Make sure to not exceed junction_max_v2 during acceleration
            # During S-Curve acceleration, the actual speed can overshoot
            # (start_v + accel * t) by (accel * t / (6 * sqrt(3)))
            a_ref.limit_accel(
                    min((junction_accel_limit_v2 - a_ref.max_start_v2) / (
                        2 * a_ref.combined_d), accel.max_accel)
                    , accel.jerk)
            a_ref.combined_d += accel.move.move_d

            combined_max_end_v2 = a_ref.calc_max_v2()
            combined_min_end_time = self.calc_min_accel_end_time(a_ref
                    , min(combined_max_end_v2, max_cruise_v2))
            if combined_min_end_time + EPSILON < accel.min_end_time:
                accel.limit_accel(a_ref.max_accel, a_ref.jerk)
                accel.max_end_v2 = combined_max_end_v2
                accel.min_end_time = combined_min_end_time
                accel.combined_d = a_ref.combined_d
                accel.start_accel = a_m
            # Also make sure to not exceed max_cruise_v2 by the end of the
            # combined_d acceleration combined so far.
            a_ref.limit_accel(
                    min((max_cruise_v2 * (53. / 54.) - a_ref.max_start_v2) / (
                        2 * a_ref.combined_d), accel.max_accel)
                    , accel.jerk)

# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        velocity = min(speed, toolhead.max_velocity)
        self.cmove = toolhead.cmove
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        self.max_accel = toolhead.max_accel
        self.max_accel_to_decel = toolhead.max_accel_to_decel
        self.max_jerk = toolhead.max_jerk
        if move_d < .000000001:
            # Extrude only move
            self.end_pos = (start_pos[0], start_pos[1], start_pos[2],
                            end_pos[3])
            axes_d[0] = axes_d[1] = axes_d[2] = 0.
            self.move_d = move_d = abs(axes_d[3])
            self.max_accel = self.max_accel_to_decel = self.max_jerk = 99999999.
            velocity = speed
            self.is_kinematic_move = False
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.
        self.max_cruise_v2 = velocity**2
        self.prev_move = None
        self.junction_max_v2 = 0.
    def limit_speed(self, speed, accel, jerk=None):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.max_accel = min(self.max_accel, accel)
        self.max_accel_to_decel = min(self.max_accel_to_decel, accel)
        if jerk and jerk < self.max_jerk:
            self.max_jerk = jerk
    def calc_peak_v2(self, accel, decel):
        if accel.accel_order == 2:
            effective_accel = min(accel.max_accel, decel.max_accel)
            peak_v2 = (accel.max_start_v2 + decel.max_start_v2
                    + 2. * self.move_d * effective_accel) * .5
            return peak_v2
        total_d = accel.combined_d + decel.combined_d - self.move_d
        high_v = math.sqrt(max(accel.max_end_v2, decel.max_end_v2))
        low_v = 0
        while high_v - low_v > EPSILON:
            guess_v = (high_v + low_v) * 0.5
            accel_d = accel.calc_min_accel_dist(guess_v)
            decel_d = decel.calc_min_accel_dist(guess_v)
            if accel_d <= accel.combined_d and decel_d <= decel.combined_d and (
                    accel_d + decel_d <= total_d):
                low_v = guess_v
            else:
                high_v = guess_v
        return low_v * low_v
    def calc_junction(self, prev_move):
        self.prev_move = prev_move
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extruder to calculate its maximum junction
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        # Find max velocity using "approximated centripetal velocity"
        axes_d = self.axes_d
        prev_axes_d = prev_move.axes_d
        junction_cos_theta = -((axes_d[0] * prev_axes_d[0]
                                + axes_d[1] * prev_axes_d[1]
                                + axes_d[2] * prev_axes_d[2])
                               / (self.move_d * prev_move.move_d))
        if junction_cos_theta > 0.999999:
            return
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos_theta))
        R = (self.toolhead.junction_deviation * sin_theta_d2
             / (1. - sin_theta_d2))
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5*(1.0+junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.max_accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.max_accel)
        self.junction_max_v2 = min(
            R * self.max_accel, R * prev_move.max_accel,
            move_centripetal_v2, prev_move_centripetal_v2, extruder_v2)
    def move(self):
        # Determine move velocities
        self.start_accel_v = self.accel.start_accel_v
        # Determine the effective accel and decel
        self.effective_accel = self.accel.effective_accel
        self.effective_decel = self.decel.effective_accel
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = self.accel.accel_t
        self.accel_offset_t = self.accel.accel_offset_t
        self.total_accel_t = self.accel.total_accel_t
        self.decel_t = self.decel.accel_t
        self.decel_offset_t = self.decel.accel_offset_t
        self.total_decel_t = self.decel.total_accel_t
        self.cruise_t = (self.move_d
                - self.accel.accel_d - self.decel.accel_d) / self.cruise_v
        # Only used to track smootheness, can be deleted
        if self.accel_t:
            self.start_v = self.start_accel_v + self.effective_accel * self.accel_offset_t
        else:
            self.start_v = self.cruise_v - self.effective_decel * self.decel_offset_t
        if self.decel_t or self.cruise_t:
            self.end_v = self.cruise_v - self.effective_decel * (self.decel_offset_t + self.decel_t)
        else:
            self.end_v = self.start_v + self.effective_accel * self.accel_t
        if self.cruise_t < -EPSILON:
            raise error(
                    'Logic error: impossible move ms_v=%.3lf, mc_v=%.3lf'
                    ', me_v=%.3lf, accel_d = %.3lf, decel_d = %.3lf'
                    ' with move_d=%.3lf, accel=%.3lf, decel=%.3lf'
                    ', jerk=%.3lf' % (self.start_v, self.cruise_v, self.end_v
                        , self.accel.accel_d, self.decel.accel_d , self.move_d
                        , self.accel.max_accel, self.decel.max_accel
                        , self.accel.jerk))
        if self.prev_move and abs(self.prev_move.end_v
                - self.start_v) > 0.0001:
            raise error('Logic error: velocity jump from %.6lf to %.6lf'
                    % (self.prev_move.end_v, self.start_v))
        self.prev_move = None
        # Generate step times for the move
        next_move_time = self.toolhead.get_next_move_time()
        if self.is_kinematic_move:
            self.toolhead.move_fill(
                self.cmove, next_move_time,
                self.accel_t, self.accel_offset_t, self.total_accel_t,
                self.cruise_t,
                self.decel_t, self.decel_offset_t, self.total_decel_t,
                self.start_pos[0], self.start_pos[1], self.start_pos[2],
                self.axes_d[0], self.axes_d[1], self.axes_d[2],
                self.start_accel_v, self.cruise_v,
                self.effective_accel, self.effective_decel,
                self.toolhead.accel_compensation)
            self.toolhead.kin.move(next_move_time, self)
        if self.axes_d[3]:
            self.toolhead.extruder.move(next_move_time, self)
        self.toolhead.update_move_time(
            self.accel_t + self.cruise_t + self.decel_t)

def reset_accel_decel(move):
    move.accel = Acceleration(move, move.max_accel)
    move.decel = Acceleration(move, move.max_accel)
def reset_smoothed_accel_decel(move):
    move.accel = Acceleration(move, move.max_accel_to_decel)
    move.decel = Acceleration(move, move.max_accel_to_decel)

class Trapezoid:
    def __init__(self):
        self.accel_queue = []
        self.decel_queue = []
    def add_decel(self, move):
        self.decel_queue.append(move)
        return True
    def add_accel(self, move):
        if self.decel_queue:
            return False
        self.accel_queue.append(move)
        return True
    def calc_peak_v2(self):
        assert self.decel_queue
        first_decel_m = self.decel_queue[0]
        if self.accel_queue and first_decel_m != self.accel_queue[-1]:
            last_accel_m = self.accel_queue[-1]
            peak_v2 = min(first_decel_m.decel.max_end_v2
                    , last_accel_m.accel.max_end_v2)
            return min(peak_v2, first_decel_m.junction_max_v2
                    , first_decel_m.max_cruise_v2
                    , last_accel_m.max_cruise_v2)
        move = first_decel_m
        peak_v2 = move.calc_peak_v2(move.accel, move.decel)
        return min(peak_v2, move.max_cruise_v2)
    def _set_decel(self, cruise_v2):
        for i in range(0, len(self.decel_queue)):
            m = self.decel_queue[i]
            m.decel.set_junction(cruise_v2, time_offset_from_start=False)
            cruise_v2 = min(cruise_v2, m.decel.start_accel.max_start_v2)
    def _set_accel(self, cruise_v2):
        i = len(self.accel_queue) - 1
        while i >= 0:
            m = self.accel_queue[i]
            m.accel.set_junction(cruise_v2, time_offset_from_start=True)
            cruise_v2 = min(cruise_v2, m.accel.start_accel.max_start_v2)
            i = self.accel_queue.index(m.accel.start_accel.move) - 1
    def setup_peak_cruise_v2(self):
        peak_cruise_v2 = self.calc_peak_v2()
        for m in self.accel_queue:
            m.max_cruise_v2 = min(m.max_cruise_v2, peak_cruise_v2)
        for m in self.decel_queue:
            m.max_cruise_v2 = min(m.max_cruise_v2, peak_cruise_v2)
    def setup_velocity_trapezoid(self):
        peak_cruise_v2 = self.calc_peak_v2()
        self._set_decel(peak_cruise_v2)
        self._set_accel(peak_cruise_v2)

class TrapezoidBuilder:
    def __init__(self):
        self.current = Trapezoid()
        self.queue = []
    def add_as_decel(self, move):
        if not self.current.add_decel(move):
            self.flush()
            self.current.add_decel(move)
    def add_as_accel(self, move):
        if not self.current.add_accel(move):
            self.flush()
            self.current.add_accel(move)
    def flush(self):
        if self.current.accel_queue or self.current.decel_queue:
            self.queue.append(self.current)
            self.current = Trapezoid()


LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class MoveQueue:
    def __init__(self):
        self.extruder_lookahead = None
        self.queue = []
        self.leftover = 0
        self.smoothed_pass_leftover = 0
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.leftover = 0
        self.smoothed_pass_leftover = 0
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def set_extruder(self, extruder):
        self.extruder_lookahead = extruder.lookahead
    def _backward_pass(self, limit, lazy, reset_accel_decel):
        queue = self.queue
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        junction_max_v2 = 0.
        decel_combiner = AccelCombiner(0.)
        non_combining_v2 = 0.
        fixed_velocity = len(queue)
        for i in range(len(queue)-1, limit-1, -1):
            move = queue[i]
            reset_accel_decel(move)
            decel = move.decel
            non_combining_decel = Acceleration(move, decel.max_accel)
            non_combining_decel.set_max_start_v2(non_combining_v2)
            decel_combiner.process_next_accel(decel, junction_max_v2)
            if lazy and (decel.max_start_v2 + EPSILON
                    > non_combining_decel.max_start_v2):
                fixed_velocity = min(i, fixed_velocity)
            non_combining_v2 = non_combining_decel.calc_max_v2()
            junction_max_v2 = min(move.junction_max_v2, move.max_cruise_v2)
        return fixed_velocity
    def _build_trapezoids(self, start, end, lazy):
        trapezoids = TrapezoidBuilder()
        if start >= end:
            return trapezoids
        queue = self.queue
        prev_move = queue[start].prev_move
        start_v2 = prev_move.decel.max_start_v2 if prev_move else 0.
        max_end_v2 = queue[start].decel.max_end_v2
        if max_end_v2 + EPSILON < start_v2:
            raise error(
                    'Logic error: impossible to reach the committed v2 = %.3lf'
                    ', max velocity = %.3lf' % (start_v2, max_end_v2))
        i = start
        accel_combiner = AccelCombiner(start_v2)
        prev_cruise_v2 = start_v2
        while i < end:
            move = queue[i]
            accel = move.accel
            decel = move.decel

            accel_combiner.process_next_accel(accel
                    , min(move.junction_max_v2, prev_cruise_v2))
            if decel.max_end_v2 > accel.max_start_v2 + EPSILON:
                # This move can accelerate
                trapezoids.add_as_accel(move)
            if accel.max_end_v2 + EPSILON > decel.max_start_v2:
                # This move must decelerate after acceleration,
                # or this is a full decel move after full accel move.
                trapezoids.add_as_decel(move)
                accel_combiner.reset(decel.start_accel.max_start_v2)
                i = queue.index(decel.start_accel.move)
            prev_cruise_v2 = queue[i].max_cruise_v2
            i += 1
        if not lazy:
            trapezoids.flush()
        return trapezoids
    def _find_peak_cruise_v2(self, start, lazy):
        end = self._backward_pass(start, lazy, reset_smoothed_accel_decel)
        trapezoids = self._build_trapezoids(start, end, lazy)
        if not trapezoids.queue:
            return start
        for trap in trapezoids.queue:
            trap.setup_peak_cruise_v2()
        assert trapezoids.queue[-1].decel_queue
        decel_queue = trapezoids.queue[-1].decel_queue
        last_move = decel_queue[-1].decel.start_accel.move
        return self.queue.index(last_move) + 1
    def _plan_moves(self, start, end, lazy):
        queue = self.queue
        if start >= end:
            return 0
        end = min(end, self._backward_pass(start, lazy, reset_accel_decel))
        trapezoids = self._build_trapezoids(start, end, lazy)
        for trap in trapezoids.queue:
            trap.setup_velocity_trapezoid()
        if not trapezoids.queue:
            return 0
        assert trapezoids.queue[-1].decel_queue
        last_move = trapezoids.queue[-1].decel_queue[-1].decel.start_accel.move
        return self.queue.index(last_move) + 1
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        queue = self.queue
        self.smoothed_pass_leftover = self._find_peak_cruise_v2(
                self.smoothed_pass_leftover, lazy)
        flush_count = self._plan_moves(self.leftover
                , self.smoothed_pass_leftover, lazy)
        if not flush_count:
            return
        # Allow extruder to do its lookahead
        move_count = self.extruder_lookahead(queue, flush_count, lazy)
        if not move_count:
            return
        # Generate step times for all moves ready to be flushed
        for move in queue[:move_count]:
            move.move()
        # Remove processed moves from the queue
        self.leftover = flush_count - move_count
        self.smoothed_pass_leftover = self.smoothed_pass_leftover - move_count
        logging.info(
                'lazy = %s, qsize = %d, smth_leftover = %d, flush_count = %d'
                , lazy, len(queue), self.smoothed_pass_leftover, flush_count)
        del queue[:move_count]
    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)

STALL_TIME = 0.100

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.150
class DripModeEndSignal(Exception):
    pass

RINGING_REDUCTION_FACTOR = 10.
MAX_ACCEL_COMPENSATION = 0.005

# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.move_queue = MoveQueue()
        self.commanded_pos = [0., 0., 0., 0.]
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        # Velocity and acceleration control
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        self.min_jerk_limit_time = config.getfloat(
                'min_jerk_limit_time', 0., minval=0.)
        if self.min_jerk_limit_time:
            max_jerk_default = self.max_accel * 6 / (
                    self.min_jerk_limit_time * RINGING_REDUCTION_FACTOR)
        else:
            max_jerk_default = self.max_accel * 30.0
        self.max_jerk = config.getfloat('max_jerk', max_jerk_default, above=0.)
        self.requested_accel_to_decel = config.getfloat(
            'max_accel_to_decel', self.max_accel * 0.5, above=0.)
        self.max_accel_to_decel = self.requested_accel_to_decel
        self.square_corner_velocity = config.getfloat(
            'square_corner_velocity', 5., minval=0.)
        self.config_max_velocity = self.max_velocity
        self.config_max_accel = self.max_accel
        self.config_square_corner_velocity = self.square_corner_velocity
        self.junction_deviation = 0.
        self.accel_compensation = config.getfloat(
            'accel_compensation', 0., minval=0., maxval=MAX_ACCEL_COMPENSATION)
        self._calc_junction_deviation()
        # Print time tracking
        self.buffer_time_low = config.getfloat(
            'buffer_time_low', 1.000, above=0.)
        self.buffer_time_high = config.getfloat(
            'buffer_time_high', 2.000, above=self.buffer_time_low)
        self.buffer_time_start = config.getfloat(
            'buffer_time_start', 0.250, above=0.)
        self.move_flush_time = config.getfloat(
            'move_flush_time', 0.050, above=0.)
        self.print_time = 0.
        self.special_queuing_state = "Flushed"
        self.need_check_stall = -1.
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.move_queue.set_flush_time(self.buffer_time_high)
        self.last_print_start_time = 0.
        self.idle_flush_print_time = 0.
        self.print_stall = 0
        self.drip_completion = None
        # Setup iterative solver
        ffi_main, self.ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(self.ffi_lib.move_alloc(), self.ffi_lib.free)
        self.move_fill = self.ffi_lib.move_fill
        self.accel_order = config.getchoice(
            'acceleration_order', { "2": 2, "4": 4, "6": 6 }, "2")
        self.ffi_lib.move_set_accel_order(self.cmove, self.accel_order)
        # Create kinematics class
        self.extruder = kinematics.extruder.DummyExtruder()
        self.move_queue.set_extruder(self.extruder)
        kin_name = config.get('kinematics')
        try:
            mod = importlib.import_module('kinematics.' + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)
        # SET_VELOCITY_LIMIT command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_VELOCITY_LIMIT',
                               self.cmd_SET_VELOCITY_LIMIT,
                               desc=self.cmd_SET_VELOCITY_LIMIT_help)
        gcode.register_command('M204', self.cmd_M204)
        # Load some default modules
        self.printer.try_load_module(config, "idle_timeout")
        self.printer.try_load_module(config, "statistics")
        self.printer.try_load_module(config, "manual_probe")
        self.printer.try_load_module(config, "tuning_tower")
    # Print time tracking
    def update_move_time(self, movetime):
        self.print_time += movetime
        flush_to_time = self.print_time - self.move_flush_time
        for m in self.all_mcus:
            m.flush_moves(flush_to_time)
    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        if est_print_time + self.buffer_time_start > self.print_time:
            self.print_time = est_print_time + self.buffer_time_start
            self.last_print_start_time = self.print_time
            self.printer.send_event("toolhead:sync_print_time",
                                    curtime, est_print_time, self.print_time)
    def get_next_move_time(self):
        if not self.special_queuing_state:
            return self.print_time
        if self.special_queuing_state == "Drip":
            # In "Drip" state - wait until ready to send next move
            while 1:
                if self.drip_completion.test():
                    raise DripModeEndSignal()
                curtime = self.reactor.monotonic()
                est_print_time = self.mcu.estimated_print_time(curtime)
                wait_time = self.print_time - est_print_time - DRIP_TIME
                if wait_time <= 0. or self.mcu.is_fileoutput():
                    return self.print_time
                self.drip_completion.wait(curtime + wait_time)
        # Transition from "Flushed"/"Priming" state to main state
        self.special_queuing_state = ""
        self.need_check_stall = -1.
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self._calc_print_time()
        return self.print_time
    def _full_flush(self):
        # Transition from "Flushed"/"Priming"/main state to "Flushed" state
        self.move_queue.flush()
        self.special_queuing_state = "Flushed"
        self.need_check_stall = -1.
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.move_queue.set_flush_time(self.buffer_time_high)
        self.idle_flush_print_time = 0.
        for m in self.all_mcus:
            m.flush_moves(self.print_time)
    def _flush_lookahead(self):
        if self.special_queuing_state:
            return self._full_flush()
        self.move_queue.flush()
    def get_last_move_time(self):
        self._flush_lookahead()
        if self.special_queuing_state:
            self._calc_print_time()
        return self.print_time
    def _check_stall(self):
        eventtime = self.reactor.monotonic()
        if self.special_queuing_state:
            if self.idle_flush_print_time:
                # Was in "Flushed" state and got there from idle input
                est_print_time = self.mcu.estimated_print_time(eventtime)
                if est_print_time < self.idle_flush_print_time:
                    self.print_stall += 1
                self.idle_flush_print_time = 0.
            # Transition from "Flushed"/"Priming" state to "Priming" state
            self.special_queuing_state = "Priming"
            self.need_check_stall = -1.
            self.reactor.update_timer(self.flush_timer, eventtime + 0.100)
        # Check if there are lots of queued moves and stall if so
        while 1:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
            stall_time = buffer_time - self.buffer_time_high
            if stall_time <= 0.:
                break
            if self.mcu.is_fileoutput():
                self.need_check_stall = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1., stall_time))
        if not self.special_queuing_state:
            # In main state - defer stall checking until needed
            self.need_check_stall = (est_print_time + self.buffer_time_high
                                     + 0.100)
    def _flush_handler(self, eventtime):
        try:
            print_time = self.print_time
            buffer_time = print_time - self.mcu.estimated_print_time(eventtime)
            if buffer_time > self.buffer_time_low:
                # Running normally - reschedule check
                return eventtime + buffer_time - self.buffer_time_low
            # Under ran low buffer mark - flush lookahead queue
            self._full_flush()
            if print_time != self.print_time:
                self.idle_flush_print_time = self.print_time
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER
    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)
    def set_position(self, newpos, homing_axes=()):
        self._flush_lookahead()
        self.commanded_pos[:] = newpos
        self.kin.set_position(newpos, homing_axes)
    def move(self, newpos, speed):
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.move_queue.add_move(move)
        if self.print_time > self.need_check_stall:
            self._check_stall()
    def dwell(self, delay):
        self.get_last_move_time()
        self.update_move_time(delay)
        self._check_stall()
    def motor_off(self):
        self.dwell(STALL_TIME)
        last_move_time = self.get_last_move_time()
        self.kin.motor_off(last_move_time)
        for ext in kinematics.extruder.get_printer_extruders(self.printer):
            ext.motor_off(last_move_time)
        self.printer.send_event("toolhead:motor_off", last_move_time)
        self.dwell(STALL_TIME)
        logging.debug('; Max time of %f', last_move_time)
    def wait_moves(self):
        self._flush_lookahead()
        if self.mcu.is_fileoutput():
            return
        eventtime = self.reactor.monotonic()
        while (not self.special_queuing_state
               or self.print_time >= self.mcu.estimated_print_time(eventtime)):
            eventtime = self.reactor.pause(eventtime + 0.100)
    def set_extruder(self, extruder):
        last_move_time = self.get_last_move_time()
        self.extruder.set_active(last_move_time, False)
        extrude_pos = extruder.set_active(last_move_time, True)
        self.extruder = extruder
        self.extruder.setup_accel_order(self.accel_order)
        self.move_queue.set_extruder(extruder)
        self.commanded_pos[3] = extrude_pos
    def get_extruder(self):
        return self.extruder
    def drip_move(self, newpos, speed):
        # Validate move
        move = Move(self, self.commanded_pos, newpos, speed)
        if move.axes_d[3]:
            raise homing.CommandError("Invalid drip move")
        if not move.move_d or not move.is_kinematic_move:
            return
        self.kin.check_move(move)
        speed = math.sqrt(move.max_cruise_v2)
        move_accel = move.max_accel
        # Transition to "Flushed" state and then to "Drip" state
        self._full_flush()
        self.special_queuing_state = "Drip"
        self.need_check_stall = self.reactor.NEVER
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.drip_completion = self.reactor.completion()
        # Split move into many tiny moves and queue them
        num_moves = max(1, int(math.ceil(move.min_move_t / DRIP_SEGMENT_TIME)))
        inv_num_moves = 1. / float(num_moves)
        submove_d = [d * inv_num_moves for d in move.axes_d]
        prev_pos = move.start_pos
        self._calc_print_time()
        try:
            for i in range(num_moves-1):
                next_pos = [p + d for p, d in zip(prev_pos, submove_d)]
                smove = Move(self, prev_pos, next_pos, speed)
                smove.limit_speed(speed, move_accel)
                self.move_queue.add_move(smove)
                prev_pos = next_pos
            smove = Move(self, prev_pos, move.end_pos, speed)
            smove.limit_speed(speed, move_accel)
            self.move_queue.add_move(smove)
            self.move_queue.flush()
        except DripModeEndSignal as e:
            self.move_queue.reset()
        # Return to "Flushed" state
        self._full_flush()
    def signal_drip_mode_end(self):
        self.drip_completion.complete(True)
    # Misc commands
    def stats(self, eventtime):
        for m in self.all_mcus:
            m.check_active(self.print_time, eventtime)
        buffer_time = self.print_time - self.mcu.estimated_print_time(eventtime)
        is_active = buffer_time > -60. or not self.special_queuing_state
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time, max(buffer_time, 0.), self.print_stall)
    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.move_queue.queue
        return self.print_time, est_print_time, lookahead_empty
    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        last_print_start_time = self.last_print_start_time
        buffer_time = print_time - estimated_print_time
        if buffer_time > -1. or not self.special_queuing_state:
            status = "Printing"
        else:
            status = "Ready"
        return { 'status': status, 'print_time': print_time,
                 'estimated_print_time': estimated_print_time,
                 'position': homing.Coord(*self.commanded_pos),
                 'printing_time': print_time - last_print_start_time }
    def _handle_request_restart(self, print_time):
        self.motor_off()
    def _handle_shutdown(self):
        self.move_queue.reset()
    def get_kinematics(self):
        return self.kin
    def get_max_velocity(self):
        return self.max_velocity, self.max_accel
    def get_max_axis_halt(self):
        # Determine the maximum velocity a cartesian axis could halt
        # at due to the junction_deviation setting.  The 8.0 was
        # determined experimentally.
        return min(self.max_velocity,
                   math.sqrt(8. * self.junction_deviation * self.max_accel))
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = min(self.requested_accel_to_decel,
                                      self.max_accel)
    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"
    def cmd_SET_VELOCITY_LIMIT(self, params):
        print_time = self.get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        max_velocity = gcode.get_float('VELOCITY', params, self.max_velocity,
                                       above=0.)
        max_accel = gcode.get_float('ACCEL', params, self.max_accel, above=0.)
        self.max_jerk = gcode.get_float('JERK', params, self.max_jerk, above=0.)
        square_corner_velocity = gcode.get_float(
            'SQUARE_CORNER_VELOCITY', params, self.square_corner_velocity,
            minval=0.)
        self.requested_accel_to_decel = gcode.get_float(
            'ACCEL_TO_DECEL', params, self.requested_accel_to_decel, above=0.)
        accel_order = gcode.get_int(
            'ACCEL_ORDER', params, self.accel_order, minval=2, maxval=6)
        if accel_order not in [2, 4, 6]:
            raise gcode.error(
                    "ACCEL_ORDER = %s is not a valid choice" % (accel_order,))
        if accel_order != self.accel_order:
            self.accel_order = accel_order
            self.ffi_lib.move_set_accel_order(self.cmove, accel_order)
            self.extruder.setup_accel_order(accel_order)
        self.accel_compensation = gcode.get_float(
            'ACCEL_COMPENSATION', params, self.accel_compensation, minval=0.
            , maxval=MAX_ACCEL_COMPENSATION)
        self.max_velocity = min(max_velocity, self.config_max_velocity)
        self.max_accel = min(max_accel, self.config_max_accel)
        self.square_corner_velocity = min(square_corner_velocity,
                                          self.config_square_corner_velocity)
        self._calc_junction_deviation()
        msg = ("max_velocity: %.6f max_accel: %.6f max_accel_to_decel: %.6f\n"
               "max_jerk: %.6f accel_order: %d square_corner_velocity: %.6f\n"
               "accel_compensation: %.8f\n"% (
                   self.max_velocity, self.max_accel, self.max_accel_to_decel,
                   self.max_jerk, accel_order, self.square_corner_velocity,
                   self.accel_compensation))
        self.printer.set_rollover_info("toolhead", "toolhead: %s" % (msg,))
        gcode.respond_info(msg, log=False)
    def cmd_M204(self, params):
        gcode = self.printer.lookup_object('gcode')
        if 'S' in params:
            # Use S for accel
            accel = gcode.get_float('S', params, above=0.)
        elif 'P' in params and 'T' in params:
            # Use minimum of P and T for accel
            accel = min(gcode.get_float('P', params, above=0.),
                        gcode.get_float('T', params, above=0.))
        else:
            gcode.respond_info('Invalid M204 command "%s"'
                               % (params['#original'],))
            return
        self.max_accel = min(accel, self.config_max_accel)
        self._calc_junction_deviation()

def add_printer_objects(config):
    config.get_printer().add_object('toolhead', ToolHead(config))
    kinematics.extruder.add_printer_objects(config)
