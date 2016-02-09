#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import matplotlib.pylab as pylab

from math import exp
from pid import PID
from tuning import GeneticTuner


class Circuit(object):
    """
    Circuit composed of resistor in series with voltage source.
    I = V / R
    P = IV
    """

    def __init__(self, resistance, init_voltage=0.0):
        self._source_voltage = init_voltage * 1.0
        self._resistance = resistance * 1.0
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

    def update(self, control, **kwargs):
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


class RCCircuit(object):
    """Simple RC Circuit."""

    def __init__(self, resistance, capacitance, init_voltage=0.0, t=0.0):
        self._source_voltage = init_voltage * 1.0
        self._resistance = resistance * 1.0
        self._capacitance = capacitance * 1.0
        self._tau = resistance * capacitance
        self._t = t * 1.0
        self._update()

    def set_source(self, source):
        self._source_voltage = source * 1.0
        self._update()

    def _update(self):
        self._vc = self._source_voltage * (1.0 - exp(-self._t / self._tau))

    @property
    def vc(self):
        return self._vc

    @property
    def measurement(self):
        return self.vc

    def update(self, control, **kwargs):
        self._t = kwargs["t"] * 1.0
        self.set_source(control)


def get_args():
    from argparse import ArgumentParser
    parser = ArgumentParser(description="Run simulation of R circuit.")

    parser.add_argument("--dt", default=0.01, type=float, help="Sample period (s)")
    parser.add_argument("--end-time", default=1.0, type=float, help="Number of seconds to run the sumlation for.")
    parser.add_argument("--max-p", default=5.0, type=float, help="Maximum value for P.")
    parser.add_argument("--max-i", default=5.0, type=float, help="Maximum value for I.")
    parser.add_argument("--max-d", default=0.1, type=float, help="Maximum value for D.")
    parser.add_argument("-p", type=float, help="Proportional gain")
    parser.add_argument("-i", type=float, help="Integral gain")
    parser.add_argument("-d", type=float, help="Derivative gain")
    parser.add_argument("--population", default=50, type=int, help="Number of individuals to include in the population.")
    parser.add_argument("--generations", default=1000, type=int, help="Number of generations to have.")

    circuit_group = parser.add_argument_group(title="Circuit specific args")
    circuit_group.add_argument("--resistance", default=4000.0, type=float, help="Resistance for the only resistor in the circuit.")
    circuit_group.add_argument("--capacitance", default=1 / 4000.0, type=float)
    circuit_group.add_argument("--power-setpoint", default=100.0, type=float, help="Target power used by resistor.")
    circuit_group.add_argument("--vc-setpoint", default=100.0, type=float, help="Target voltage across the capacitor.")
    circuit_group.add_argument("--output", default="tmp.png", type=str, help="Output filename")

    return parser.parse_args()


def main2():
    args = get_args()

    pylab.clf()

    resistance = args.resistance
    capacitance = args.capacitance
    generations = args.generations
    population = args.population
    dt = args.dt
    end_time = args.end_time
    setpoint = args.vc_setpoint

    tuner = GeneticTuner(population, end_time, dt, RCCircuit,
                         system_args=(resistance, capacitance),
                         max_p=args.max_p,
                         max_i=args.max_i,
                         max_d=args.max_d,
                         )
    Kp = args.p
    Ki = args.i
    Kd = args.d
    if not (Kp and Ki and Kd):
        Kp_, Ki_, Kd_ = tuner.find_gains(setpoint, iterations=generations)
        Kp = Kp or Kp_
        Ki = Ki or Ki_
        Kd = Kd or Kd_
    fitness = tuner.fitness((Kp, Ki, Kd), setpoint)

    c = RCCircuit(resistance, capacitance, init_voltage=0)
    pid = PID(Kp=Kp, Ki=Ki, Kd=Kd)

    vc = []
    t = xrange(int(end_time / dt))
    for i in t:
        pid.update(c.measurement, setpoint, dt=dt)
        c.update(pid.control, t=i * dt)
        #c.update(setpoint, t=i * dt)
        vc.append(c.measurement)

    pylab.plot(map(lambda x: x * dt, t), vc)
    pylab.xlabel("Time")
    pylab.ylabel("Vc")
    pylab.title("Kp:{}, Ki:{}, Kd:{}".format(Kp, Ki, Kd))
    pylab.savefig(args.output)
    print("-p {} -i {} -d {}".format(Kp, Ki, Kd))
    print("fitness:{}".format(fitness))

    return 0

def main():
    args = get_args()

    pylab.clf()

    resistance = args.resistance
    generations = args.generations
    population = args.population
    dt = args.dt
    end_time = args.end_time
    setpoint = args.power_setpoint

    tuner = GeneticTuner(population, end_time / 4, dt, Circuit,
                         system_args=(resistance, ),
                         max_p=args.max_p,
                         max_i=args.max_i,
                         max_d=args.max_d,
                         )
    Kp = args.p
    Ki = args.i
    Kd = args.d
    if not (Kp and Ki and Kd):
        Kp_, Ki_, Kd_ = tuner.find_gains(setpoint, iterations=generations)
        Kp = Kp or Kp_
        Ki = Ki or Ki_
        Kd = Kd or Kd_
    fitness = tuner.fitness((Kp, Ki, Kd), setpoint)

    c = Circuit(resistance)
    pid = PID(Kp=Kp, Ki=Ki, Kd=Kd)

    power = []
    voltage = []
    t = xrange(int(end_time / dt))
    for i in t:
        if i < end_time / dt / 2:
            pid.update(c.power, setpoint, dt=dt)
        else:
            pid.update(c.power, setpoint / 2, dt=dt)
        c.set_source(pid.control)
        voltage.append(pid.control)
        power.append(c.power)

    pylab.plot(map(lambda x: x * dt, t), power)
    pylab.xlabel("Time")
    pylab.ylabel("power")
    pylab.title("Power across {} Ohm resistor. (Setpoint of {} W)".format(resistance, setpoint))
    pylab.savefig(args.output)
    print("-p {} -i {} -d {}".format(Kp, Ki, Kd))
    print("fitness:{}".format(fitness))

    return 0


if __name__ == "__main__":
    sys.exit(main2())

