#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from random import random, uniform
from genetic import Genetic
from pid import PID


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

        self._cached_gains = None
        self._cached_population = None

    def _create_individual(self):
        """Individual is just gains."""
        return [uniform(self._min_p, self._max_p),
                uniform(self._min_i, self._max_i),
                uniform(self._min_d, self._max_d)]

    def fitness(self, individual, target):
        """
        Check the final value of running the simulation and the time
        constant tau where y is ~63.2% of the target value.
        """
        dt = self._sample_period
        tau = self._tau

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
        if random() < 0.333:
            individual[0] = uniform(self._min_p, self._max_p)
        if random() < 0.333:
            individual[1] = uniform(self._min_i, self._max_i)
        if random() < 0.333:
            individual[2] = uniform(self._min_d, self._max_d)

    def _breed(self, male, female):
        """Combine the traits of 2 different gains."""
        return [male[0], female[1], female[2]]

    def find_gains(self, target, iterations=100, retain=0.2,
                   rand_select=0.05, mutate=0.01):
        """Generate the gains."""
        fitness = self.fitness
        population = self._create_population()
        for i in xrange(iterations):
            population = self._evolve(population, target, retain,
                                      rand_select, mutate)
        gains = max(population, key=lambda x: fitness(x, target))
        self._cached_population = population
        self._cached_gains = gains
        return gains

    @property
    def cached_gains(self):
        return self._cached_gains

    @property
    def cached_population(self):
        return self._cached_population

    @staticmethod
    def _time_near(t, y, point, backwards=True):
        """Get the time near a point on graph y."""
        # Traversing backwards gives priority to earlier times
        # in the event we have 2 y values that are the same.
        if backwards:
            y = y[::-1]
            t = t[::-1]

        t_ = t[0]
        tolerance_range = max(y) - min(y)
        for i in xrange(len(y)):
            tolerance = abs(y[i] - point)
            if tolerance < tolerance_range:
                t_ = t[i]
                tolerance_range = tolerance
        return t_

