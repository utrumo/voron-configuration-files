# Event-driven sync for Nevermore Micro carbon filter fan
#
# Monitors Nevermore Mini fan (nevermore_mini_fan) and print state,
# syncs Nevermore Micro fan on state transitions:
#   Mini off -> on  : Micro ON
#   Mini on  -> off : Micro OFF
#   Print started   : Micro ON (don't wait for Mini/VOC)
#
# Between events, speed is not touched — manual override via
# SET_FAN_SPEED / Mainsail slider persists until next event.
#
# Installed as symlink: ~/klipper/klippy/extras/nevermore_micro_sync.py
# Source: ~/printer_data/config/nevermore_micro_sync.py
#
# Config example:
#   [nevermore_micro_sync]
#   pin: PA8
#   fan_name: nevermore_micro
#   watch_fan: nevermore_mini_fan
#   poll_interval: 1.0

import logging
from . import fan

INITIAL_PIN_DELAY = 0.100

class NevermoreMicroSync:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.fan = fan.Fan(config)
        self.fan_name = config.get('fan_name', 'nevermore_micro')
        self.printer.add_object('fan_generic %s' % self.fan_name, self)

        self.watch_fan_name = config.get('watch_fan', 'nevermore_mini_fan')
        self.poll_interval = config.getfloat('poll_interval', 1.0, minval=0.5)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_FAN_SPEED", "FAN",
                                   self.fan_name,
                                   self.cmd_SET_FAN_SPEED,
                                   desc=self.cmd_SET_FAN_SPEED_help)

        self.watch_fan = None
        self.last_speed = -1.0
        self.prev_mini_on = False
        self.prev_printing = False

    def handle_ready(self):
        self.watch_fan = self.printer.lookup_object(
            'fan_generic %s' % self.watch_fan_name, None)
        if self.watch_fan is None:
            logging.error("nevermore_micro_sync: watch_fan '%s' not found",
                          self.watch_fan_name)
            return
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback,
                               reactor.monotonic() + INITIAL_PIN_DELAY)
        logging.info("nevermore_micro_sync: started, watching '%s', "
                     "poll %.1fs", self.watch_fan_name, self.poll_interval)

    def _set_speed(self, speed):
        self.fan.set_speed(speed)
        self.last_speed = speed

    def cmd_SET_FAN_SPEED(self, gcmd):
        speed = gcmd.get_float('SPEED', 0.)
        self._set_speed(max(0., min(1., speed)))

    def callback(self, eventtime):
        if self.watch_fan is None:
            return eventtime + self.poll_interval

        # Read Mini fan state
        try:
            mini_status = self.watch_fan.get_status(eventtime)
            mini_on = mini_status.get('speed', 0.) > 0.
        except Exception:
            mini_on = False

        # Read print state
        try:
            print_stats = self.printer.lookup_object('print_stats')
            ps_status = print_stats.get_status(eventtime)
            printing = ps_status.get('state') in ('printing', 'paused')
        except Exception:
            printing = False

        # Detect transitions and act
        if not self.prev_printing and printing:
            self._set_speed(1.0)
        if not self.prev_mini_on and mini_on:
            self._set_speed(1.0)
        if self.prev_mini_on and not mini_on:
            self._set_speed(0.0)

        self.prev_mini_on = mini_on
        self.prev_printing = printing

        return eventtime + self.poll_interval

    def get_status(self, eventtime):
        return {
            'speed': max(0., self.last_speed),
            'rpm': None,
        }

def load_config(config):
    return NevermoreMicroSync(config)
