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


from random import uniform, random, choice
from operator import add
class GeneticTuner(object):
    """Tuner that uses a genetic algorithm."""

    def __init__(self, target, size, min_val, max_val, iterations=100):
        self._size = size  # Population size
        self._min_val = min_val
        self._max_val = max_val
        self._target = target

        self._Kp = None
        self._Ki = None
        self._Kd = None
        self._find_Ks(iterations)

    def _create_individual(self, min_val, max_val):
        """Create a member of the population."""
        return [uniform(min_val, max_val),
                uniform(min_val, max_val),
                uniform(min_val, max_val)]

    def _create_population(self, min_v, max_v):
        """Create a population of individuals."""
        create = self._create_individual
        return [create(min_v, max_v) for x in xrange(self._size)]

    def _fitness(self, individual, target):
        """Get the fitness of an individual."""
        # ALL OF THIS NEEDS TO BE ABSTRACTED OUT L8R
        steps = 100
        resistance = self._target

        Kp, Ki, Kd = individual
        c = Circuit(resistance)
        pid = PID(Kp=Kp, Ki=Ki, Kd=Kd)

        for i in xrange(steps):
            pid.update(c.power, target)
            voltage = pid.control
            c.set_source(voltage)

        current = c.power
        return abs(target - current)

    def _grade(self, population, target):
        """Average fitness for a population."""
        fitness = self._fitness
        total = reduce(add, [fitness(x, target) for x in population], 0)
        return (total * 1.0) / len(population)

    def _evolve(self, population, target, retain=0.2, rand_select=0.05,
                mutate=0.01):
        """
        Evolve a population

        retain:
            Top percentage of the population to retain for the next
            generation.
        rand_select:
            Probability that an individual not retained will be retained
            to promote genetic diversity.
        mutate:
            Probability that a retained individual will have a new
            property.
        """
        fitness = self._fitness
        sorted_pop = sorted(population, key=lambda x: fitness(x, target))
        retain_count = int(len(population) * retain)
        parents = sorted_pop[:retain_count]

        # Randomly add others to promote diversity
        parents += filter(lambda x: random() < rand_select,
                          sorted_pop[retain_count:])

        # Mutate some parents
        mutate_field = 1.0 / len(population[0])
        min_v = self._min_val
        max_v = self._max_val
        for individual in parents:
            if random() < mutate:
                # Chance to mutate each value
                if random() < mutate_field:
                    individual[0] = uniform(min_v, max_v)
                if random() < mutate_field:
                    individual[1] = uniform(min_v, max_v)
                if random() < mutate_field:
                    individual[2] = uniform(min_v, max_v)

        # Breed between parents to create children until the previous
        # population size is reached.
        shortage = len(population) - len(parents)
        children = []
        while len(children) < shortage:
            male = choice(parents)
            female = choice(parents)
            if male != female:
                # Parents cannot be the same
                # Child will just have half of each parent's genes
                children.append([male[0], female[1], female[2]])
        return parents + children

    def _find_Ks(self, iterations):
        """Generate the Ks."""
        fitness = self._fitness
        target = self._target
        population = self._create_population(self._min_val, self._max_val)
        for i in xrange(iterations):
            population = self._evolve(population, target)
        best = max(population, key=lambda x: fitness(x, target))
        self._Kp, self._Ki, self._Kd = best

    @property
    def Kp(self):
        return self._Kp

    @property
    def Ki(self):
        return self._Ki

    @property
    def Kd(self):
        return self._Kd


def main(Kp=1, Ki=0.01, Kd=0.0):
    pylab.clf()

    resistance = 4000.0
    steps = 1000
    setpoint = 100  # Watts

    c = Circuit(resistance)
    pid = PID(Kp=Kp, Ki=Ki, Kd=Kd)

    power = []
    voltage = []
    for i in xrange(steps):
        pid.update(c.power, setpoint)
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
    t = GeneticTuner(4000.0, 100, 0, 2)
    sys.exit(main(Kp=t.Kp, Ki=t.Ki, Kd=t.Kd))

