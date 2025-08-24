# Virtual extruder that works with virtual thermistor sensors
#
# This module provides an extruder implementation that can work with
# virtual temperature sensors instead of requiring physical sensors.
#
# Usage:
#   [virtual_extruder]
#   step_pin: PB1
#   dir_pin: !PB0
#   enable_pin: !PD6
#   sensor: v_hotend          # reference to temperature_sensor section
#   control: watermark
#   microsteps: 16
#   rotation_distance: 33.683
#   nozzle_diameter: 0.400
#   filament_diameter: 1.750
#   min_temp: 0
#   max_temp: 250
#   min_extrude_temp: 0
#   max_extrude_only_distance: 50.0
#   pressure_advance: 0.1
#   pressure_advance_smooth_time: 0.01
#
# Copyright (C) 2024  Virtual Klipper Printer Project
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class VirtualExtruder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()

        # Get extruder kinematics configuration
        self.step_pin = config.get('step_pin')
        self.dir_pin = config.get('dir_pin')
        self.enable_pin = config.get('enable_pin')
        self.microsteps = config.getint('microsteps', 16)
        self.rotation_distance = config.getfloat('rotation_distance', 33.5)
        self.nozzle_diameter = config.getfloat('nozzle_diameter', 0.4)
        self.filament_diameter = config.getfloat('filament_diameter', 1.75)
        self.max_extrude_only_distance = config.getfloat('max_extrude_only_distance', 50.0)
        self.pressure_advance = config.getfloat('pressure_advance', 0.0)
        self.pressure_advance_smooth_time = config.getfloat('pressure_advance_smooth_time', 0.0)

        # Get heater configuration
        self.max_power = config.getfloat('max_power', 1.0)
        self.min_temp = config.getfloat('min_temp', 0)
        self.max_temp = config.getfloat('max_temp', 250)
        self.min_extrude_temp = config.getfloat('min_extrude_temp', 170)

        # Get sensor reference
        sensor_name = config.get('sensor')
        if not sensor_name:
            raise config.error("sensor parameter must be specified")

        # Store sensor name for later lookup
        self.sensor_name = sensor_name
        self.sensor = None

        # Setup virtual heater control
        class VirtualHeaterPin:
            def __init__(self):
                self.pwm_value = 0.0

            def set_pwm(self, pwm):
                self.pwm_value = max(0.0, min(1.0, pwm))

            def get_pwm(self, eventtime):
                return self.pwm_value

            def setup_max_duration(self, duration):
                pass  # No physical constraints for virtual pin

        self.heater = VirtualHeaterPin()

        # Setup temperature control
        control_type = config.get('control', 'watermark')
        if control_type == 'watermark':
            self.temp_control = WatermarkControl(config, self)
        elif control_type == 'pid':
            self.temp_control = PIDControl(config, self)
        else:
            raise config.error(f"Unknown control type '{control_type}'")

        # Register with printer
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M104", self.cmd_M104)
        gcode.register_command("M109", self.cmd_M109)

    def _handle_connect(self):
        # Look up the temperature sensor now that all objects are initialized
        if self.sensor is None:
            try:
                sensor_obj = self.printer.lookup_object(self.sensor_name)
                if not hasattr(sensor_obj, 'get_temp') or not hasattr(sensor_obj, 'get_status'):
                    raise Exception(
                        f"'{self.sensor_name}' is not a valid temperature sensor")
                self.sensor = sensor_obj
            except Exception as e:
                import logging
                logging.error(
                    f"Failed to find temperature sensor '{self.sensor_name}': {e}")
                self.sensor = None

        self.temp_control.handle_connect()

    def get_temp(self, eventtime):
        if self.sensor is None:
            return (0.0, False)  # Return safe default if sensor not available
        return self.sensor.get_temp(eventtime)

    def get_status(self, eventtime):
        temp, fault = self.get_temp(eventtime)
        return {
            'temperature': temp,
            'target': self.temp_control.get_target_temp(),
            'power': self.heater.get_pwm(eventtime),
        }

    def get_pwm(self):
        # Expose current PWM power for external consumers
        try:
            return float(getattr(self.heater, 'pwm_value', 0.0))
        except Exception:
            return 0.0

    def set_temp(self, temp, wait=False):
        if temp and (temp < self.min_temp or temp > self.max_temp):
            raise self.printer.command_error(
                f"Requested temperature ({temp}) is outside of the configured "
                f"range ({self.min_temp} to {self.max_temp})")
        self.temp_control.set_target_temp(temp, wait)

    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        self.set_temp(temp, wait)

    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)


class WatermarkControl:
    def __init__(self, config, heater):
        self.heater = heater
        self.target_temp = 0.
        self.heater_max_power = heater.max_power

    def handle_connect(self):
        pass

    def get_target_temp(self):
        return self.target_temp

    def set_target_temp(self, temp, wait=False):
        self.target_temp = temp
        if temp <= 0:
            self.heater.heater.set_pwm(0.)
        else:
            self.heater.heater.set_pwm(self.heater_max_power)


class PIDControl:
    def __init__(self, config, heater):
        self.heater = heater
        self.Kp = config.getfloat('pid_Kp', 63.0)
        self.Ki = config.getfloat('pid_Ki', 1.5)
        self.Kd = config.getfloat('pid_Kd', 270.0)
        self.target_temp = 0.
        self.last_error = 0.
        self.integral = 0.

    def handle_connect(self):
        pass

    def get_target_temp(self):
        return self.target_temp

    def set_target_temp(self, temp, wait=False):
        self.target_temp = temp
        # Simple PID implementation
        if temp <= 0:
            self.heater.heater.set_pwm(0.)
            self.integral = 0.
            self.last_error = 0.
        else:
            current_temp, _ = self.heater.get_temp(0)
            error = temp - current_temp

            # Proportional term
            p_term = self.Kp * error

            # Integral term
            self.integral += error
            i_term = self.Ki * self.integral

            # Derivative term
            d_term = self.Kd * (error - self.last_error)
            self.last_error = error

            # Calculate PWM (0.0 to 1.0)
            pwm = (p_term + i_term + d_term) / 100.0
            pwm = max(0.0, min(1.0, pwm))

            self.heater.heater.set_pwm(pwm)


def load_config(config):
    return VirtualExtruder(config)
