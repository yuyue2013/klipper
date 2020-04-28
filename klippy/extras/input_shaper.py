# Kinematic input shaper to minimize motion vibrations in XY plane
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import chelper

class InputShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.damping_ratio_x = config.getfloat('damping_ratio_x',
                                               0., minval=0.,
                                               maxval=1.)
        self.damping_ratio_y = config.getfloat('damping_ratio_y',
                                               0., minval=0.,
                                               maxval=1.)
        self.spring_period_x = config.getfloat('spring_period_x', 0., minval=0.)
        self.spring_period_y = config.getfloat('spring_period_y', 0., minval=0.)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.shapers = {'zv': ffi_lib.INPUT_SHAPER_ZV
                , 'zvd': ffi_lib.INPUT_SHAPER_ZVD
                , 'zvdd': ffi_lib.INPUT_SHAPER_ZVDD
                , 'zvddd': ffi_lib.INPUT_SHAPER_ZVDDD
                , 'ei': ffi_lib.INPUT_SHAPER_EI
                , '2hump_ei': ffi_lib.INPUT_SHAPER_2HUMP_EI}
        self.shaper_type = config.getchoice('type', self.shapers, 'zvd')
        self.stepper_kinematics = []
        self.orig_stepper_kinematics = []
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_INPUT_SHAPER",
                               self.cmd_SET_INPUT_SHAPER,
                               desc=self.cmd_SET_INPUT_SHAPER_help)
    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        kin = self.toolhead.get_kinematics()
        # Lookup stepper kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        steppers = kin.get_steppers()
        for s in steppers:
            sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
            orig_sk = s.set_stepper_kinematics(sk)
            res = ffi_lib.input_shaper_set_sk(sk, orig_sk)
            if res < 0:
                s.set_stepper_kinematics(orig_sk)
                continue
            s.set_trapq(self.toolhead.get_trapq())
            self.stepper_kinematics.append(sk)
            self.orig_stepper_kinematics.append(orig_sk)
        # Configure initial values
        self.old_delay = 0.
        self._set_input_shaper(self.damping_ratio_x, self.damping_ratio_y,
                               self.spring_period_x, self.spring_period_y,
                               self.shaper_type)
    def _get_step_generation_window(self, ffi_lib, shaper_type
                                    , damped_spring_period):
        if shaper_type == ffi_lib.INPUT_SHAPER_ZV:
            return .25 * damped_spring_period
        if shaper_type == ffi_lib.INPUT_SHAPER_ZVD:
            return .5 * damped_spring_period
        if shaper_type == ffi_lib.INPUT_SHAPER_ZVDD:
            return .75 * damped_spring_period
        if shaper_type == ffi_lib.INPUT_SHAPER_ZVDDD:
            return damped_spring_period
        if shaper_type == ffi_lib.INPUT_SHAPER_EI:
            return .5 * damped_spring_period
        if shaper_type == ffi_lib.INPUT_SHAPER_2HUMP_EI:
            return .75 * damped_spring_period
        raise self.gcode.error(
            "Shaper type '%d' is not supported" % (shaper_type))
    def _set_input_shaper(self, damping_ratio_x, damping_ratio_y
                          , spring_period_x, spring_period_y, shaper_type):
        if shaper_type != self.shaper_type:
            self.toolhead.flush_step_generation()
        damped_spring_period_x = spring_period_x / math.sqrt(
                1. - damping_ratio_x**2)
        damped_spring_period_y = spring_period_y / math.sqrt(
                1. - damping_ratio_y**2)
        ffi_main, ffi_lib = chelper.get_ffi()
        new_delay = self._get_step_generation_window(ffi_lib, shaper_type
                , max(damped_spring_period_x, damped_spring_period_y))
        self.toolhead.note_step_generation_scan_time(new_delay,
                                                     old_delay=self.old_delay)
        self.damping_ratio_x = damping_ratio_x
        self.damping_ratio_y = damping_ratio_y
        self.spring_period_x = spring_period_x
        self.spring_period_y = spring_period_y
        self.shaper_type = shaper_type
        for sk in self.stepper_kinematics:
            ffi_lib.input_shaper_set_shaper_params(sk
                    , damped_spring_period_x, damped_spring_period_y
                    , damping_ratio_x, damping_ratio_y, shaper_type)
    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"
    def cmd_SET_INPUT_SHAPER(self, params):
        gcode = self.printer.lookup_object('gcode')
        damping_ratio_x = gcode.get_float('DAMPING_RATIO_X', params,
                                         self.damping_ratio_x, minval=0.,
                                         maxval=1.)
        damping_ratio_y = gcode.get_float('DAMPING_RATIO_Y', params,
                                         self.damping_ratio_y, minval=0.,
                                         maxval=1.)
        spring_period_x = gcode.get_float('SPRING_PERIOD_X', params,
                                          self.spring_period_x, minval=0.)
        spring_period_y = gcode.get_float('SPRING_PERIOD_Y', params,
                                          self.spring_period_y, minval=0.)
        shaper_type = self.shaper_type
        if 'TYPE' in params:
            shaper_type_str = gcode.get_str('TYPE', params).lower()
            if shaper_type_str not in self.shapers:
                raise self.gcode.error(
                    "Requested shaper type '%s' is not supported" % (
                        shaper_type_str))
            shaper_type = self.shapers[shaper_type_str]
        self._set_input_shaper(damping_ratio_x, damping_ratio_y,
                               spring_period_x, spring_period_y, shaper_type)
        gcode.respond_info("damping_ratio_x:%.9f damping_ratio_y:%.9f "
                           "spring_period_x:%.9f spring_period_y:%.9f "
                           "shaper_type: %s" % (
                               damping_ratio_x, damping_ratio_y
                               , spring_period_x, spring_period_y
                               , self.shapers.keys()[
                                   self.shapers.values().index(shaper_type)]))

def load_config(config):
    return InputShaper(config)
