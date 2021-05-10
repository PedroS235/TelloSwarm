import itertools

import numpy as np
import pygame

from capt import CAPTSolver

pygame.init()

WIDTH = 640
HEIGHT = 480
FPS = 30
BACKGROUND_COLOR = pygame.Color('white')


class GraphicalSolver(CAPTSolver):
    """
    This class is a subclass of CAPTSolver and contains all the methods to run the simulation.
    """

    START_COLOR = pygame.Color('red')
    GOAL_COLOR = pygame.Color('green')
    AXIS_COLOR = pygame.Color('blue')
    ASSIGNMENT_COLOR = pygame.Color('magenta')
    TRAJECTORY_COLOR = pygame.Color('black')

    def __init__(self, width, height):
        self._scale_factor = 100
        self._origin = np.array([width // 2, height // 2])

        # noinspection PyTypeChecker
        super(GraphicalSolver, self).__init__(strict=False)

    def _dist_from_pixels(self, d):
        return d / self._scale_factor

    def _dist_to_pixels(self, d):
        return round(d * self._scale_factor)

    def _coord_from_pixels(self, x, y):
        # noinspection PyTypeChecker
        return (np.array([x, y]) - self._origin) * [1, -1] / self._scale_factor

    def _coord_to_pixels(self, x, y):
        return (np.array([x, y]) * self._scale_factor * [1, -1] + self._origin).astype(int)

    def render(self, surface):
        t = pygame.time.get_ticks() / 1000
        # print(t, ' ' * 20, end='\r')
        pygame.draw.line(surface, self.AXIS_COLOR, self._coord_to_pixels(0, 0), self._coord_to_pixels(1, 0))
        pygame.draw.line(surface, self.AXIS_COLOR, self._coord_to_pixels(0, 0), self._coord_to_pixels(0, 1))
        for start in self.starts:
            pygame.draw.circle(surface, self.START_COLOR, self._coord_to_pixels(*start), self._dist_to_pixels(self.radius))
            pygame.draw.circle(surface, self.START_COLOR, self._coord_to_pixels(*start),
                               self._dist_to_pixels(self.radius * (2 ** .5)), 1)
        for goal in self.goals:
            pygame.draw.circle(surface, self.GOAL_COLOR, self._coord_to_pixels(*goal), self._dist_to_pixels(self.radius))
        if self.assignment is not None:
            for i, j in enumerate(self.assignment):
                if j != -1:
                    pygame.draw.line(surface, self.ASSIGNMENT_COLOR,
                                     self._coord_to_pixels(*self.starts[i]), self._coord_to_pixels(*self.goals[j]))
        if self.trajectory is not None:
            for i, _ in enumerate(self.starts):
                pos = self.trajectory(i, t)
                pygame.draw.circle(surface, self.TRAJECTORY_COLOR,
                                   self._coord_to_pixels(*pos), self._dist_to_pixels(self.radius), 1)

    def add_start(self, pos):
        super(GraphicalSolver, self).add_start(self._coord_from_pixels(*pos))

    def add_goal(self, pos):
        super(GraphicalSolver, self).add_goal(self._coord_from_pixels(*pos))


if __name__ == '__main__':
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    solver = GraphicalSolver(WIDTH, HEIGHT)
    super(GraphicalSolver, solver).add_start((1, 1))
    super(GraphicalSolver, solver).add_goal((1.5, 1))
    super(GraphicalSolver, solver).add_start((0.5, 0.9))
    super(GraphicalSolver, solver).add_goal((2, 0.9))
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                exit()
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1:
                    solver.add_start(e.pos)
                else:
                    solver.add_goal(e.pos)
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    solver.clear()
                elif e.key == pygame.K_RETURN:
                    t0 = pygame.time.get_ticks() / 1000
                    solver.compute_trajectories(t0)
        screen.fill(BACKGROUND_COLOR)
        solver.render(screen)
        pygame.display.flip()
        clock.tick(FPS)
