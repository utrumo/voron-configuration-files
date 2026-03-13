# PI controller for driver fan — tracks TMC2240 temperatures
#
# Replaces [controller_fan] approach with temperature-based control.
# Uses temperature_combined-like logic but avoids its TMC2240
# incompatibility (TMC2240 returns temperature=None before homing).
#
# Creates own fan.Fan (like controller_fan.py) for direct pin control
# from reactor timer — fan_generic methods depend on toolhead flush
# which doesn't happen when idle.
#
# Installed as symlink: ~/klipper/klippy/extras/driver_fan_controller.py
# Source: ~/printer_data/config/driver_fan_controller.py
#
# Config example:
#   [driver_fan_controller]
#   pin: PD14
#   cycle_time: 0.00005
#   sensors: tmc2240 stepper_x, tmc2240 stepper_y
#   target_temp: 65.0

import logging
from . import fan

PIN_MIN_TIME = 0.100
WARMUP_DURATION = 5.0

class DriverFanController:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.fan = fan.Fan(config)
        self.sensor_names = config.getlist('sensors')
        self.target = config.getfloat('target_temp', 65.0)
        self.kp = config.getfloat('kp', 0.2)
        self.ki = config.getfloat('ki', 2.0)
        self.ema_alpha = config.getfloat('ema_alpha', 0.3,
                                         minval=0., maxval=1.)
        self.off_below = config.getfloat('off_below', 0.15,
                                         minval=0., maxval=1.)
        self.hysteresis = config.getfloat('hysteresis', 0.05, minval=0.)
        self.max_speed_delta = config.getfloat('max_speed_delta', 0.,
                                               minval=0., maxval=1.)
        self.poll_interval = config.getfloat('poll_interval', 0.3,
                                             minval=0.1)
        self.integral_max = config.getfloat('integral_max', 10.0)

        self.sensors = []
        self.smooth_temp = 0.0
        self._ema_initialized = False
        self.integral = 0.0
        self.last_speed = -1.0
        self.last_time = 0.0
        self._warmup_until = 0.0

    def handle_ready(self):
        self.sensors = []
        for name in self.sensor_names:
            try:
                obj = self.printer.lookup_object(name)
                self.sensors.append(obj)
            except Exception:
                logging.warning("driver_fan_controller: sensor '%s' "
                                "not found, skipping", name)
        if not self.sensors:
            logging.error("driver_fan_controller: no sensors available")
            return
        reactor = self.printer.get_reactor()
        self.last_time = reactor.monotonic()
        reactor.register_timer(self.callback,
                               reactor.monotonic() + PIN_MIN_TIME)
        logging.info("driver_fan_controller: started, %d sensors, "
                     "target %.1fC, poll %.1fs",
                     len(self.sensors), self.target, self.poll_interval)

    def callback(self, eventtime):
        # Read max temperature from all sensors
        temps = []
        for sensor in self.sensors:
            try:
                status = sensor.get_status(eventtime)
                temp = status.get('temperature')
                if temp is not None:
                    temps.append(temp)
            except Exception:
                pass
        if not temps:
            # All sensors returned None (steppers disabled) — ramp down
            self.smooth_temp = 0.0
            self._ema_initialized = False
            self._warmup_until = 0.0
            self.integral = 0.0
            speed = 0.0
            if self.last_speed > 0:
                if self.max_speed_delta > 0:
                    speed = max(0.0, self.last_speed - self.max_speed_delta)
                if speed < self.off_below:
                    speed = 0.0
                self.fan.set_speed(speed)
                self.last_speed = speed
            return eventtime + self.poll_interval

        raw_temp = max(temps)

        # EMA smoothing — first_reading = transition from None→data
        first_reading = not self._ema_initialized
        if first_reading:
            self.smooth_temp = raw_temp
            self._ema_initialized = True
        else:
            self.smooth_temp = (self.ema_alpha * raw_temp
                                + (1 - self.ema_alpha) * self.smooth_temp)

        # Warmup: steppers just enabled → 100% for WARMUP_DURATION
        if first_reading:
            self._warmup_until = eventtime + WARMUP_DURATION
            self.fan.set_speed(1.0)
            self.last_speed = 1.0
            self.last_time = eventtime
            logging.info("driver_fan_controller: warmup started, "
                         "temp %.1fC, fan 100%% for %.0fs",
                         raw_temp, WARMUP_DURATION)
            return eventtime + self.poll_interval

        if self._warmup_until > 0 and eventtime < self._warmup_until:
            # Still in warmup — keep 100%, accumulate EMA
            self.last_time = eventtime
            return eventtime + self.poll_interval

        # Warmup ended — clear flag
        warmup_ended = self._warmup_until > 0
        if warmup_ended:
            self._warmup_until = 0.0
            logging.info("driver_fan_controller: warmup ended, "
                         "temp %.1fC, PI taking over", self.smooth_temp)

        # PI controller
        dt = min(eventtime - self.last_time, self.poll_interval * 2)
        self.last_time = eventtime
        error = self.smooth_temp - self.target

        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integral_max,
                            min(self.integral_max, self.integral))
        i_term = self.ki * self.integral / 100.0

        speed = max(0.0, min(1.0, p_term + i_term))

        if 0 < speed < self.off_below:
            speed = 0.0

        # Rate limit — clamp speed change per cycle for smooth ramping
        # Skip when: warmup just ended, or fan was off (immediate response)
        if (self.max_speed_delta > 0 and self.last_speed > 0
                and not warmup_ended):
            delta = speed - self.last_speed
            if abs(delta) > self.max_speed_delta:
                speed = self.last_speed + (self.max_speed_delta
                                           if delta > 0
                                           else -self.max_speed_delta)
                speed = max(0.0, min(1.0, speed))

        # Apply hysteresis — only change if difference is significant
        if (abs(speed - self.last_speed) > self.hysteresis
                or (speed == 0 and self.last_speed > 0)
                or (speed >= 1.0 and self.last_speed < 1.0)):
            self.fan.set_speed(speed)
            self.last_speed = speed

        return eventtime + self.poll_interval

    def get_status(self, eventtime):
        return {
            'temperature': self.smooth_temp,
            'target': self.target,
            'speed': max(0.0, self.last_speed),
            'rpm': None,
            'integral': self.integral,
        }

def load_config(config):
    return DriverFanController(config)
