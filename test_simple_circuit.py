#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import matplotlib.pylab as pylab

from pid import PID


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


def main():
    pylab.clf()

    resistance = 4000.0
    steps = 10000
    setpoint = 100  # Watts

    c = Circuit(resistance)
    pid = PID()

    power = []
    voltage = []
    for i in xrange(steps):
        pid.update(c.power, setpoint)
        c.set_source(pid.control)
        voltage.append(pid.control)
        power.append(c.power)

    pylab.plot(xrange(steps), power)
    pylab.xlabel("Ticks")
    pylab.ylabel("Power")
    pylab.title("Power across {} Ohm resistor. (Setpoint of {} W)".format(resistance, setpoint))
    pylab.savefig("power.png")

    return 0


if __name__ == "__main__":
    sys.exit(main())
