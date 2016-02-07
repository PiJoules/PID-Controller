#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from random import random, uniform
from genetic import Genetic
from pid import PID


class Gains(object):
    __slots__ = ("_p", "_i", "_d")

    """
    I'm stupid for this. The main reason why I have this class is just
    because you cannot hash a list, but you can hash a tuple.
    The hashing is required in genetic.py.
    """

    def __init__(self, p, i, d):
        self._p = p
        self._i = i
        self._d = d

    def mutate(self, p, i, d):
        """Just a setter method for the gains."""
        self._p = p
        self._i = i
        self._d = d

    @classmethod
    def from_parents(cls, male, female):
        """Breed a child from parents."""
        return Gains(male.P, female.I, female.D)

    @property
    def P(self):
        return self._p

    @property
    def I(self):
        return self._i

    @property
    def D(self):
        return self._d

    @property
    def gains(self):
        return (self._p, self._i, self._d)

    def __eq__(self, other):
        return self.gains == other.gains

    def __hash__(self):
        return hash((self._p, self._i, self._d))


class GeneticTuner(Genetic):
    """
    Find the gain values of a PID controller using a
    genetic algorithm.
    """

    def __init__(self, population_size, rise_time, sample_period,
                 system_cls, system_args=None, system_kwargs=None,
                 min_p=0.0, max_p=1.0,
                 min_i=0.0, max_i=0.1,
                 min_d=0.0, max_d=0.01,
                 tau=None):
        super(GeneticTuner, self).__init__(population_size)
        self._min_p = min_p
        self._max_p = max_p
        self._min_i = min_i
        self._max_i = max_i
        self._min_d = min_d
        self._max_d = max_d

        # Desired amount of time (s) to reach setpoint
        self._rise_time = rise_time
        # Sampleing period in seconds
        self._sample_period = sample_period

        # Tau is another way to fit an individual
        self._tau = tau

        # System to simulate when finding gains.
        # This system will be created every time when finding fitness
        # for an individual.
        self._system_cls = system_cls
        self._system_args = system_args or []
        self._system_kwargs = system_kwargs or {}

    def _create_individual(self):
        """Individual is just gains."""
        #return Gains(uniform(self._min_p, self._max_p),
        #             uniform(self._min_i, self._max_i),
        #             uniform(self._min_d, self._max_d))
        return [uniform(self._min_p, self._max_p),
                uniform(self._min_i, self._max_i),
                uniform(self._min_d, self._max_d)]

    def _fitness(self, individual, target):
        """
        Check the final value of running the simulation and the time
        constant tau where y is ~63.2% of the target value.
        """
        dt = self._sample_period
        tau = self._tau

        #pid = PID(Kp=individual.P,
        #          Ki=individual.I,
        #          Kd=individual.D)
        pid = PID(Kp=individual[0],
                  Ki=individual[1],
                  Kd=individual[2])

        system = self._system_cls(*self._system_args, **self._system_kwargs)

        t = range(0, self._rise_time, dt)
        y = []
        for i in t:
            output = system.measurement
            pid.update(output, target, dt=dt)
            y.append(output)
            system.update(pid.control)

        if tau is not None:
            t632 = self._time_near(t, y, 0.632 * target)
            tau_diff = abs(tau - t632)
        else:
            tau_diff = 0

        return abs(target - y[-1]) + tau_diff

    def _mutate_individual(self, individual):
        """Randomly change any of the gains."""
        #Kp, Ki, Kd = individual.gains
        Kp, Ki, Kd = individual
        if random() < 0.333:
            Kp = uniform(self._min_p, self._max_p)
        if random() < 0.333:
            Ki = uniform(self._min_i, self._max_i)
        if random() < 0.333:
            Kd = uniform(self._min_d, self._max_d)
        #individual.mutate(Kp, Ki, Kd)
        individual[0] = Kp
        individual[1] = Ki
        individual[2] = Kd

    def _breed(self, male, female):
        """Combine the traits of 2 different gains."""
        #return Gains.from_parents(male, female)
        return [male[0], female[1], female[2]]

    def find_gains(self, target, iterations=100, retain=0.2,
                   rand_select=0.05, mutate=0.01):
        """Generate the gains."""
        fitness = self._fitness
        population = self._create_population()
        for i in xrange(iterations):
            population = self._evolve(population, target, retain,
                                      rand_select, mutate)
        return max(population, key=lambda x: fitness(x, target))

    @staticmethod
    def _time_near(t, y, point, backwards=True):
        """Get the time near a point on graph y."""
        # Traversing backwards gives priority to earlier times
        # in the event we have 2 y values that are the same.
        if backwards:
            y = y[::-1]
            t = t[::-1]

        t_ = None
        tolerance_range = max(y) - min(y)
        for i in xrange(len(y)):
            tolerance = abs(y[i] - point)
            if tolerance < tolerance_range:
                t_ = t[i]
                tolerance_range = tolerance
        return t_
