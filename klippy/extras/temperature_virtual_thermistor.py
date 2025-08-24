# Virtual thermistor temperature sensor for Klipper
#
# This module provides a simulated temperature sensor that can be linked to
# heaters (extruder, bed) to provide realistic temperature curves.
#
# Usage:
#   [temperature_sensor v_hotend]
#   sensor_type: virtual_thermistor
#   # optional tuning parameters:
#   ambient: 25.0           # °C ambient temperature
#   watts: 40.0             # effective heater power in watts
#   tau: 15.0               # thermal time constant in seconds
#   noise: 0.2              # ±°C random noise (0 = off)
#   sample_rate: 10.0       # update rate in Hz
#   heater: extruder        # optional: link to specific heater
#
# Then reference from a heater:
#   [extruder]
#   sensor_type: temperature_sensor
#   sensor_pin: v_hotend
#
# Copyright (C) 2024  Virtual Klipper Printer Project
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import random


class VirtualThermistor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Configuration parameters
        self.ambient = config.getfloat('ambient', 25.0)
        self.watts = config.getfloat('watts', 40.0)
        self.tau = max(0.1, config.getfloat('tau', 15.0))
        self.noise = max(0.0, config.getfloat('noise', 0.0))
        self.sample_rate = max(0.5, config.getfloat('sample_rate', 10.0))
        # optional: link to a heater by name (e.g., "extruder",
        # "heater_bed"). If set, the thermistor will read that
        # heater's power to drive the temperature model.
        self.heater_name = config.get('heater', None)

        # State variables
        self.temp = self.ambient
        self._last_t = None
        self._heater = None
        self._timer = None

        # Temperature sensor interface
        self.min_temp = self.max_temp = 0.0
        self._callback = None

        # Register with printer
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    # Temperature sensor API methods
    def setup_callback(self, temperature_callback):
        """Set up temperature update callback."""
        self._callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        """Set temperature limits."""
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        """Return the time between temperature reports."""
        return 1.0 / self.sample_rate

    def get_temp(self, eventtime):
        """Return current temperature and fault status."""
        return self.temp, False

    def get_status(self, eventtime):
        """Return status information."""
        return {
            'temperature': round(self.temp, 2),
        }

    # Internal methods
    def _handle_connect(self):
        """Called when printer connects - start the simulation."""
        # Link to heater if specified
        if self.heater_name:
            try:
                # Look up via heaters registry so names like 'extruder' /
                # 'heater_bed' resolve to Heater instances.
                pheaters = self.printer.lookup_object('heaters')
                self._heater = pheaters.lookup_heater(self.heater_name)
            except Exception:
                self._heater = None
        # Cache MCU reference to avoid per-tick lookup
        try:
            self._mcu = self.printer.lookup_object('mcu')
        except Exception:
            self._mcu = None

        # Start periodic updates
        self._last_t = self.reactor.monotonic()
        period = 1.0 / self.sample_rate
        self._timer = self.reactor.register_timer(
            self._update, self._last_t + period)

    def _update(self, eventtime):
        """Update temperature simulation."""
        if self._last_t is None:
            self._last_t = eventtime
            return eventtime + (1.0 / self.sample_rate)

        dt = max(0.0, eventtime - self._last_t)
        self._last_t = eventtime

        # Get heater power (0.0 to 1.0) from the Heater's reported status
        power = 0.0
        if self._heater is not None:
            try:
                status = self._heater.get_status(eventtime)
                power = float(status.get('power', 0.0))
            except Exception:
                power = 0.0

        # Clamp power to valid range
        power = max(0.0, min(1.0, power))

        # First-order thermal model
        # dT/dt = (target_temp - current_temp) / tau
        target_temp = self.ambient + self.watts * power
        self.temp += (target_temp - self.temp) * (dt / self.tau)

        # Clamp temperature to reasonable bounds
        self.temp = max(-20.0, min(400.0, self.temp))

        # Add noise if configured
        if self.noise > 0.0:
            self.temp += (random.random() * 2.0 - 1.0) * self.noise

        # Call temperature callback if set
        if self._callback is not None and self._mcu is not None:
            self._callback(
                self._mcu.estimated_print_time(eventtime), self.temp)

        # Schedule next update
        return eventtime + (1.0 / self.sample_rate)


def load_config(config):
    """Load configuration and register sensor factory."""
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("virtual_thermistor", VirtualThermistor)
