import itertools

import numpy as np


class CAPTSolver:
  """
  This class is responsible to compute the trajectories the the drones need to follow to arrive at the desired destination
  """
  def __init__(self, radius=0.1, max_speed=0.2, logger=print, strict=True):
    """
    Constructor of the class CAPTSolver
    parameters:
      - radius = radius of the drone (dimension of the drone) in meters
      - max_speed = the maximal speed that the drone can go
      - strict = ?
    """

    self.radius = radius
    self.max_speed = max_speed
    self.starts = []
    self.goals = []
    self.assignment = None
    self.start_time = None
    self.end_time = None
    self.trajectory = None
    self.logger = logger
    self.strict = strict

  def _fail(self, etype, emsg):
    """
      This method raises an error
      parameters:
        - etype = the type of the error
        - emsg = the msg to show
    """

    self.logger(emsg)
    if self.strict:
      raise etype(emsg)

  def _collides_with(self, pos, nodes, safety_radius_factor=1.):
    """
      This method checks if the given position of a node is close to another.
      parameters:
        - pos = (x, y) position of the node to compare with the other nodes
        - nodes = [(x, y),(x, y),...] contains both targets and start nodes

      returns the node that the node at position pos is touching
      returns None if the node at position pos does not collide with any other node
    """

    for node in nodes:
      d = np.linalg.norm(node - pos) #computes the distance between the 2 points
      if d < 2 * safety_radius_factor * self.radius:
        return node
    return None

  def add_start(self, pos):
    """
      This method adds a new start node to the start array
      parameters:
        - pos = position to add a start node
      If a there exists a node at the position pos a exeption will be raised
    """

    self.logger(f"{self}: adding start {pos}")
    pos = np.array(pos)
    other = self._collides_with(pos, self.starts + self.goals)
    if other is not None:
      self._fail(ValueError, f"{self}: start {pos} collides with {other} ")
      return

    other = self._collides_with(pos, self.starts, safety_radius_factor=(2 ** .5))
    if other is not None:
      self._fail(ValueError, f"{self}: cannot guarantee collision-free paths, start locations too near: {pos} (new) and {other} (old)")
      return

    self.starts.append(pos)

  def add_goal(self, pos):
    """
      This method adds a new goal/target node to the goals array
      parameters:
        - pos = position to add a start node
      If a there exists a node at the position pos a exeption will be raised
    """
    self.logger(f"{self}: adding goal {pos}")
    pos = np.array(pos)
    other = self._collides_with(pos, self.starts + self.goals)
    if other is not None:
      self._fail(ValueError, f"{self}: goal {pos} collides with {other}")
      return

    self.goals.append(pos)

def clear(self):
    """
      This method restarts all the values. Acts like a RESET
    """
    self.starts = []
    self.goals = []
    self.assignment = None
    self.start_time = None
    self.end_time = None
    self.trajectory = None

  def assigned(self, i):
    """
      This method the node that is at the index i
      parameters:
        - i = index of a node
    """
    if i >= len(self.assignment) or self.assignment[i] == -1:
      return self.starts[i]
    return self.goals[self.assignment[i]]

def compute_trajectories(self, t0):
    """
      This method is the algorithm to compute the trajectories
    """
    self.logger(f"{self}: computing trajectories")

    if not self._solve_assignments():
      self._fail(RuntimeError, f"{self}: error occurred during assignement")
      return

    # determine final time
    distances = np.array([np.linalg.norm(start - self.assigned(i)) for i, start in enumerate(self.starts)])
    flight_duration = max(distances) / self.max_speed

    def traj(i, t):
      start = self.starts[i]
      goal = self.assigned(i)
      t1 = max(min((t - t0) / flight_duration, 1), 0)
      # noinspection PyTypeChecker
      return (1 - t1) * start + t1 * goal

    self.start_time = t0
    self.end_time = t0 + flight_duration
    self.speeds = distances / flight_duration
    self.trajectory = traj
    self.logger(f"{self}: computed trajectories for t in [{self.start_time};{self.end_time}]")

  def _solve_assignments(self):
    """
      This method contains the algorithm that solves the assigments, meaning sets the drones goals.
    """
    self.logger("{self}: solving assignments")
    N = len(self.starts)
    M = len(self.goals)

    if not (N and M):
      self._fail(ValueError, f"{self}: no start/end locations ({N}, M)")
      return False

    # compute distance squared matrix
    D = np.zeros((N, M))
    for i in range(N):
      for j in range(M):
        D[i, j] = np.linalg.norm(self.starts[i] - self.goals[j]) ** 2

    # bruteforce search, minimize total distance squared
    best = (np.inf, None)
    goals_idx = list(range(M)) + [-1] * (N - M)
    for assignment in itertools.permutations(goals_idx, N):
      cost = 0
      for i, j in enumerate(assignment):
        if j != -1:
          cost += D[i, j]
      if cost < best[0]:
        best = (cost, assignment)
    self.assignment = best[1]
    self.logger(f"{self}: found assignement {self.assignment}")
    return best[0]
