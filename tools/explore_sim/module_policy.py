"""Wraps the rover's frontier_planner module as a simulator policy, so the
simulator exercises the exact code the explorer node runs.

The simulator indexes cells (row down, col right); the module uses the
OccupancyGrid layout (i up, j right). The belief is flipped and poses are
converted to metres with the map origin at (0, 0)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.environ.get('ROVER_EXPLORE_PKG', os.path.join(HERE, '..', '..', 'src', 'rover_explore'))
sys.path.insert(0, PKG)
from rover_explore.frontier_planner import GridMap, Params, Planner  # noqa: E402

from sim import RES  # noqa: E402


class Module:
    name = 'rover_explorer'

    def __init__(self, interval=2.0, name=None, **kw):
        self.interval = interval
        self.params = Params(**kw)
        if name:
            self.name = name

    def reset(self, sim):
        self.pl = Planner(self.params)
        self.end_reason = ''
        self.excluded = None
        self.H = sim.shape[0]

    def to_xy(self, r, c):
        return (c + 0.5) * RES, (self.H - 1 - r + 0.5) * RES

    def decide(self, sim, event):
        gm = GridMap(np.flipud(sim.belief), RES, 0.0, 0.0)
        x, y = self.to_xy(*sim.pos)
        ag = self.to_xy(*sim.goal) if sim.goal is not None else None
        d = self.pl.decide(gm, (x, y, sim.heading), event, ag)
        self.excluded = np.flipud(self.pl._mask_from_xy(gm, self.pl.excluded_xy))
        if d.kind == 'done':
            self.end_reason = d.reason
            return 'done', None
        if d.kind in ('keep', 'wait'):
            return 'keep', self.interval
        i, j = gm.to_cell(d.goal[0], d.goal[1])
        return (self.H - 1 - i, j), self.interval
