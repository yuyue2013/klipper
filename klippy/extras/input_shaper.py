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
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.sk = self.orig_sk = None
        self.damping_ratio = config.getfloat('damping_ratio', 0.,
                                             minval=0., maxval=1.)
        self.spring_period = config.getfloat('spring_period', 0., minval=0.)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.shapers = {'zv': ffi_lib.INPUT_SHAPER_ZV
                , 'zvd': ffi_lib.INPUT_SHAPER_ZVD
                , 'zvdd': ffi_lib.INPUT_SHAPER_ZVDD
                , 'zvddd': ffi_lib.INPUT_SHAPER_ZVDDD
                , 'ei': ffi_lib.INPUT_SHAPER_EI
                , '2hump_ei': ffi_lib.INPUT_SHAPER_2HUMP_EI}
        self.shaper_type = config.getchoice('type', self.shapers, 'zvd')
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_INPUT_SHAPER", "AXIS", self.name,
                                   self.cmd_SET_INPUT_SHAPER,
                                   desc=self.cmd_SET_INPUT_SHAPER_help)
    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        kin = self.toolhead.get_kinematics()
        # Lookup stepper kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        steppers = kin.get_steppers()
        for s in steppers:
            if s.get_name() != self.name:
                continue
            self.sk = sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
            self.orig_sk = orig_sk = s.set_stepper_kinematics(sk)
            res = ffi_lib.input_shaper_set_sk(sk, orig_sk)
            if res < 0:
                s.set_stepper_kinematics(orig_sk)
                continue
            s.set_trapq(self.toolhead.get_trapq())
        if self.sk is None:
            raise self.printer.config_error(
                "No matching stepper '%s' found" % (self.name,))
        # Configure initial values
        self.old_delay = 0.
        self._set_input_shaper(self.damping_ratio, self.spring_period,
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
    def _set_input_shaper(self, damping_ratio, spring_period, shaper_type):
        if shaper_type != self.shaper_type:
            self.toolhead.flush_step_generation()
        damped_spring_period = spring_period / math.sqrt(
                1. - damping_ratio**2)
        ffi_main, ffi_lib = chelper.get_ffi()
        new_delay = self._get_step_generation_window(ffi_lib, shaper_type
                , damped_spring_period)
        self.toolhead.note_step_generation_scan_time(new_delay,
                                                     old_delay=self.old_delay)
        self.damping_ratio = damping_ratio
        self.spring_period = spring_period
        self.shaper_type = shaper_type
        ffi_lib.input_shaper_set_shaper_params(self.sk
                , damped_spring_period, damping_ratio, shaper_type)
    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"
    def cmd_SET_INPUT_SHAPER(self, params):
        gcode = self.printer.lookup_object('gcode')
        damping_ratio = gcode.get_float('DAMPING_RATIO', params,
                                        self.damping_ratio,
                                        minval=0., maxval=1.)
        spring_period = gcode.get_float('SPRING_PERIOD', params,
                                        self.spring_period, minval=0.)
        shaper_type = self.shaper_type
        if 'TYPE' in params:
            shaper_type_str = gcode.get_str('TYPE', params).lower()
            if shaper_type_str not in self.shapers:
                raise self.gcode.error(
                    "Requested shaper type '%s' is not supported" % (
                        shaper_type_str))
            shaper_type = self.shapers[shaper_type_str]
        self._set_input_shaper(damping_ratio, spring_period, shaper_type)
        gcode.respond_info("damping_ratio:%.9f spring_period:%.9f "
                           "shaper_type: %s" % (
                               damping_ratio , spring_period
                               , self.shapers.keys()[
                                   self.shapers.values().index(shaper_type)]))

def load_config_prefix(config):
    return InputShaper(config)
