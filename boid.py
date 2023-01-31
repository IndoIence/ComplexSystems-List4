import pygame as pg
from random import uniform
from vehicle import Vehicle


class Boid(Vehicle):

    # CONFIG
    debug = False
    min_speed = .01
    max_speed = .2
    max_force = 1
    max_turn = 5
    perception = 60
    crowding = 15
    can_wrap = False
    blind = True
    edge_distance_pct = 5

    ###############

    def __init__(self):
        Boid.set_boundary(Boid.edge_distance_pct)

        # Randomize starting position and velocity
        start_position = pg.math.Vector2(
            uniform(0, Boid.max_x),
            uniform(0, Boid.max_y))
        start_velocity = pg.math.Vector2(
            uniform(-1, 1) * Boid.max_speed,
            uniform(-1, 1) * Boid.max_speed)

        super().__init__(start_position, start_velocity,
                         Boid.min_speed, Boid.max_speed,
                         Boid.max_force, Boid.can_wrap)

        self.rect = self.image.get_rect(center=self.position)

        self.debug = Boid.debug

    def separation(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            dist = self.position.distance_to(boid.position)
            if dist < self.crowding:
                steering -= boid.position - self.position
        steering = self.clamp_force(steering)
        return steering

    def alignment(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.velocity
        steering /= len(boids)
        steering -= self.velocity
        steering = self.clamp_force(steering)
        return steering / 8

    def cohesion(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.position
        steering /= len(boids)
        steering -= self.position
        steering = self.clamp_force(steering)
        return steering / 100

    def update(self, dt, boids):
        sequence = [self.separation, self.alignment, self.cohesion]

        steering = pg.Vector2()

        if not self.can_wrap:
            steering += self.avoid_edge()

        neighbors = self.get_neighbors(boids)
        if neighbors:

            for command in sequence:
                change = command(neighbors)
                steering += change
                super().update(dt/3, steering)

            # separation = self.separation(neighbors)
            # steering += separation
            # super().update(dt/3, steering)
            # alignment = self.alignment(neighbors)
            # steering += alignment
            # super().update(dt/3, steering)
            # cohesion = self.cohesion(neighbors)
            # steering += cohesion
            # super().update(dt/3, steering)
            return

            # separation = self.separation(neighbors)
            # alignment = self.alignment(neighbors)
            # cohesion = self.cohesion(neighbors)
            # steering += separation + alignment + cohesion
            # DEBUG
            # separation *= 0
            # alignment *= 0
            # cohesion *= 0



        # steering = self.clamp_force(steering)

        super().update(dt, steering)

    def get_neighbors(self, boids):
        neighbors = []
        for boid in boids:
            if boid != self:
                dist = self.position.distance_to(boid.position)
                if dist < self.perception:
                    if not self.blind:
                        neighbors.append(boid)
                    else:
                        dot_pr = pg.Vector2.dot(self.velocity,(self.position - boid.position))
                        if dot_pr > 0:
                            neighbors.append(boid)


        return neighbors
