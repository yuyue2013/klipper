# Positional smoother on cartesian XY axes
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, types
import chelper

def calc_smooth_t(toolhead):
    return (4. * (math.sqrt(2.) + 1.)
            * toolhead.square_corner_velocity / toolhead.max_accel)

MAX_ACCEL_COMPENSATION = 0.005

class SmoothAxis:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.accel_comp_x = config.getfloat('accel_comp_x'
                , 0., minval=0., maxval=MAX_ACCEL_COMPENSATION)
        self.accel_comp_y = config.getfloat('accel_comp_y'
                , 0., minval=0., maxval=MAX_ACCEL_COMPENSATION)
        self.stepper_kinematics = []
        self.orig_stepper_kinematics = []
        # Stepper kinematics code
        ffi_main, ffi_lib = chelper.get_ffi()
        self._set_time = ffi_lib.smooth_axis_set_time
        self._set_sk = ffi_lib.smooth_axis_set_sk
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_SMOOTH_AXIS",
                               self.cmd_SET_SMOOTH_AXIS,
                               desc=self.cmd_SET_SMOOTH_AXIS_help)
    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        kin = self.toolhead.get_kinematics()
        # Lookup stepper kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        steppers = kin.get_steppers()
        for s in steppers:
            sk = ffi_main.gc(ffi_lib.smooth_axis_alloc(), ffi_lib.free)
            orig_sk = s.set_stepper_kinematics(sk)
            res = ffi_lib.smooth_axis_set_sk(sk, orig_sk)
            if res < 0:
                s.set_stepper_kinematics(orig_sk)
                continue
            s.set_trapq(self.toolhead.get_trapq())
            self.stepper_kinematics.append(sk)
            self.orig_stepper_kinematics.append(orig_sk)
        # Configure initial values
        self.smooth_t = 0.
        self._set_smooth_time(calc_smooth_t(self.toolhead))
        self._set_accel_comp(self.accel_comp_x, self.accel_comp_y)
        # Intercept square_corner_velocity changes
        def calc_junction_deviation(toolhead):
            self._set_smooth_time(calc_smooth_t(toolhead))
        self.toolhead._calc_junction_deviation = types.MethodType(
                calc_junction_deviation, self.toolhead)
        # Calculate corner_velocity changes
        def calc_junction_max_v2(toolhead, prev_move, move,
                                 sin_theta_d2, tan_theta_d2):
            X = 0.25 * self.smooth_t * (tan_theta_d2 / sin_theta_d2 - 1.0)
            return min(X * move.accel, X * prev_move.accel)**2
        self.toolhead.calc_junction_max_v2 = types.MethodType(
                calc_junction_max_v2, self.toolhead)
    def _set_smooth_time(self, smooth_t):
        old_smooth_time = self.smooth_t * .5
        new_smooth_time = smooth_t * .5
        self.toolhead.note_step_generation_scan_time(new_smooth_time,
                                                     old_delay=old_smooth_time)
        self.smooth_t = smooth_t
        ffi_main, ffi_lib = chelper.get_ffi()
        for sk in self.stepper_kinematics:
            ffi_lib.smooth_axis_set_time(sk, smooth_t, smooth_t)
    def _set_accel_comp(self, accel_comp_x, accel_comp_y):
        self.toolhead.flush_step_generation()
        self.accel_comp_x = accel_comp_x
        self.accel_comp_y = accel_comp_y
        ffi_main, ffi_lib = chelper.get_ffi()
        for sk in self.stepper_kinematics:
            ffi_lib.smooth_axis_set_accel_comp(sk, accel_comp_x, accel_comp_y)
    cmd_SET_SMOOTH_AXIS_help = "Set cartesian time smoothing parameters"
    def cmd_SET_SMOOTH_AXIS(self, params):
        gcode = self.printer.lookup_object('gcode')
        accel_comp_x = gcode.get_float('ACCEL_COMP_X', params, self.accel_comp_x,
                                   minval=0., maxval=MAX_ACCEL_COMPENSATION)
        accel_comp_y = gcode.get_float('ACCEL_COMP_Y', params, self.accel_comp_y,
                                   minval=0., maxval=MAX_ACCEL_COMPENSATION)
        self._set_accel_comp(accel_comp_x, accel_comp_y)
        gcode.respond_info("accel_comp_x:%.9f accel_comp_y:%.9f" % (
            accel_comp_x, accel_comp_y))

def load_config(config):
    return SmoothAxis(config)
