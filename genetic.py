#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from operator import add
from random import random, choice


class Genetic(object):
    """Genetic algorithm or something. I don't know."""

    def __init__(self, population_size):
        self._population_size = population_size

    def _create_individual(self):
        """
        Create an individual for the population.

        return:
            Individual object.
        """
        raise NotImplementedError

    def _fitness(self, individual, target):
        """
        Get the fitness of an individual.

        individual:
            The individual to check.
        target:
            The desired target individual.
        return:
            Non-negative number indicating how different the individual
            is from the target.
            Zero means the individual is the exact same as the target.
            High value means the individual is different from the target.
        """
        raise NotImplementedError

    def _mutate_individual(self, individual):
        """
        Mutate an individual.

        individual:
            The individual to mutate inplace.
        return:
            None
        """
        raise NotImplementedError

    def _breed(self, male, female):
        """
        Create a child individual whose genes are some combination of
        the mothers' and fathers' genes.

        male:
            Father individual.
        female:
            Mother individual.
        return:
            Child individual.
        """
        raise NotImplementedError

    def _create_population(self):
        """Create a population of individuals."""
        individual = self._create_individual
        return [individual() for x in xrange(self._population_size)]

    def grade(self, population, target):
        """Average fitness for a population."""
        fitness = self._fitness
        total = reduce(add, [fitness(x, target) for x in population], 0)
        return (total * 1.0) / len(population)

    def _evolve(self, population, target, retain, rand_select, mutate):
        """
        Evolve a population

        population:
            Population to evolve.
        target:
            Target individual to reach.
        retain:
            Top percentage of the population to retain for the next
            generation.
        rand_select:
            Probability that an individual not retained will be retained
            to promote genetic diversity.
        mutate:
            Probability that a retained individual will have a new
            property.
        return:
            A population representing the next generation after the given one.
        """
        # Localize methods
        fitness = self._fitness
        mutate_individual = self._mutate_individual
        breed = self._breed

        retain_count = int(len(population) * retain)
        if retain_count < 2:
            raise RuntimeError("There must be at least 2 retained individuals in the population to breed.")
        sorted_pop = sorted(population, key=lambda x: fitness(x, target))
        parents = sorted_pop[:retain_count]
        ignored = sorted_pop[retain_count:]

        # Randomly add others to promote diversity
        parents += filter(lambda x: random() < rand_select, ignored)

        # Mutate some parents
        for individual in parents:
            if random() < mutate:
                mutate_individual(individual)

        # Breed between parents to create children until the previous
        # population size is reached.
        children = []
        shortage = self._population_size - len(parents)
        while len(children) < shortage:
            male = choice(parents)
            female = choice(parents)
            children.append(breed(male, female))

        return parents + children

