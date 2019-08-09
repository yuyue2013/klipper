# Support for custom board pin aliases
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class PrinterBoardAliases:
    def __init__(self, config, chip_name):
        ppins = config.get_printer().lookup_object('pins')
        pin_resolver = ppins.get_pin_resolver(chip_name)
        aliases = config.get("aliases")
        parts = [a.split('=', 1) for a in aliases.split(',')]
        for alias, pin in parts:
            pin = pin.strip()
            if pin.startswith('<') and pin.endswith('>'):
                # Just a comment
                continue
            pin_resolver.alias_pin(alias.strip(), pin)

def load_config(config):
    return PrinterBoardAliases(config, "mcu")

def load_config_prefix(config):
    return PrinterBoardAliases(config, config.get_name().split()[1])
