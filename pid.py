#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function


class PID(object):
    """Discrete PID Controller."""

    def __init__(self, Kp=1.0, Ki=0.01, Kd=0.0, error_init=0,
                 control_max=None, control_min=None):
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._control = None

        self._last_error = error_init

        control = error_init
        if control_max is not None and control > control_max:
            control = control_max
        if control_min is not None and control < control_min:
            control = control_min
        self._integral = control

        self._control_max = control_max
        self._control_min = control_min

    @property
    def control(self):
        """Control input of the system."""
        return self._control

    def update(self, measurement, setpoint, dt=1.0):
        """Update the control variable."""
        # Localize variables
        last_error = self._last_error
        integral = self._integral
        control_max = self._control_max
        control_min = self._control_min
        Kp = self._Kp
        Ki = self._Ki
        Kd = self._Kd

        # Prep variables for finding PID
        error = setpoint - measurement
        derivative = (error - last_error) / dt
        integral = (error * dt) + integral

        # Find PID
        control = (Kp * error) + (Ki * integral) + (Kd * derivative)
        if control_max is not None and control > control_max:
            control = control_max
        if control_min is not None and control < control_min:
            control = control_min

        # Save state
        self._control = control
        self._last_error = error
        self._integral = integral

