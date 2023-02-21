
from random import random
import numpy as np
from numpy.linalg import norm

maxSpeed = 4
maxForce = 0.01

class Boid:
    def __init__(self, wind_x, wind_y):
        self.bounds = [wind_x, wind_y]
        self.Pos = np.array([[random()*(wind_x-10)+10],
                            [random()*(wind_y-10)+10]])
        
        self.Vel = np.array([[random()-0.5],
                            [random()-0.5]])
        
        self.Acc = np.array([[0.0],
                             [0.0]])
        
        self.perception = 100
        
    def seperate(self, boids):
        total = 0
        steering = np.array([[0.0],
                             [0.0]])
        
        for b in boids:
            d = norm(b.Pos - self.Pos)
            if d < self.perception and b != self:
                diff = self.Pos - b.Pos
                diff /= (d**2)
                steering += diff
                total += 1

        if total > 0:
            steering /= total
            steering /= norm(steering)
            steering *= maxSpeed
            steering -= self.Vel
            steering.clip(-maxForce, maxForce)

        return steering

    def align(self, boids):
        total = 0
        steering = np.array([[0.0],
                             [0.0]])

        for b in boids:
            if norm(b.Pos - self.Pos) < self.perception and b is not self:
                steering += (b.Vel / norm(b.Vel))
                total += 1

        if total > 0:
            steering /= total
            steering /= norm(steering)
            steering *= maxSpeed
            steering -= (self.Vel / norm(self.Vel))
            steering.clip(-maxForce, maxForce)

        return steering

    def cohesion(self, boids):
        total = 0
        steering = np.array([[0.0],
                             [0.0]])

        for b in boids:
            if norm(b.Pos - self.Pos) < self.perception/2 and b != self:
                steering += b.Pos
                total += 1

        if total > 0:
            steering /= total
            steering -= self.Pos
            steering /= norm(steering)
            steering *= maxSpeed
            steering -= self.Vel
            steering.clip(-maxForce, maxForce)

        return steering

    def update(self, boids):
        self.Acc = np.array([[0.0],
                             [0.0]])
        self.Acc += 0.1*self.seperate(boids)
        self.Acc += 0.1*self.align(boids)
        self.Acc += 0.1*self.cohesion(boids)

        self.Vel.clip(-maxForce, maxForce)
        self.Vel += self.Acc
        self.Pos += self.Vel


        if self.Pos[0] >= self.bounds[0]:
            self.Pos[0] = 0
        elif self.Pos[0] < 0:
            self.Pos[0] = self.bounds[0]

        if self.Pos[1] >= self.bounds[1]:
            self.Pos[1] = 0
        elif self.Pos[1] < 0:
            self.Pos[1] = self.bounds[1]

        return self.Pos
