#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import matplotlib.pylab as pylab

from pid import PID
from tuning import GeneticTuner


class Circuit(object):
    """
    Circuit composed of resistor in series with voltage source.
    I = V / R
    P = IV
    """

    def __init__(self, resistance, init_voltage=0.0):
        self._source_voltage = init_voltage
        self._resistance = resistance
        self._update()

    @property
    def current(self):
        """Current across resistor."""
        return self._current

    @property
    def power(self):
        """Power used by resistor."""
        return self._power

    def set_source(self, voltage):
        """Set the source voltage and update the state of the circuit."""
        self._source_voltage = voltage
        self._update()

    def set_resistance(self, resistance):
        """Set the resistance and update the state."""
        self._resistance = resistance
        self._update()

    def _update(self):
        """Update the state of the circuit."""
        self._current = self._source_voltage / self._resistance
        self._power = self._source_voltage * self._current

    def update(self, control):
        """
        Update the system based on the control.
        Used by GeneticTuner object.
        """
        self.set_source(control)

    @property
    def measurement(self):
        """
        Property of the system we want to measure.
        Used by GeneticTuner object.
        """
        return self.power


def main():
    pylab.clf()

    resistance = 4000.0
    generations = 3000
    population = 10
    steps = 400
    setpoint = 100  # Watts

    tuner = GeneticTuner(population, generations, 1, Circuit,
                         system_args=(resistance, ))
    Kp, Ki, Kd = tuner.find_gains(setpoint)

    c = Circuit(resistance)
    pid = PID(Kp=Kp, Ki=Ki, Kd=Kd)

    power = []
    voltage = []
    for i in xrange(steps):
        if i < steps / 2:
            pid.update(c.power, setpoint)
        else:
            pid.update(c.power, setpoint / 2)
        c.set_source(pid.control)
        voltage.append(pid.control)
        power.append(c.power)

    pylab.plot(xrange(steps), power)
    pylab.xlabel("Time")
    pylab.ylabel("Power")
    pylab.title("Power across {} Ohm resistor. (Setpoint of {} W)".format(resistance, setpoint))
    pylab.savefig("power.png")
    print(Kp, Ki, Kd)

    return 0


if __name__ == "__main__":
    sys.exit(main())

