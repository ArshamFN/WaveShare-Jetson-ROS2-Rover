"""Frontier exploration simulator on a real occupancy map.

Ground truth: a saved SLAM map. Free cells (254) are free; occupied (0) and
never-seen (205) cells are solid. The simulated robot carries a 360 degree,
12 m lidar (500 rays, like the RPLIDAR C1 at 10 Hz) and builds its own belief
map from scratch. Navigation mimics the rover's Nav2 setup: NavFn on the
belief map with allow_unknown, a 0.25 m goal tolerance, the rover's inscribed
radius for obstacle clearance, 0.22 m/s driving, 0.6 rad/s spin-in-place when
the heading error exceeds 0.5 rad, 0.15 m xy goal tolerance, a 1 Hz replan
while driving, and a fixed time penalty when a goal fails.

Exploration policies plug in through Policy.decide(); see policies.py.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from PIL import Image
from scipy import ndimage
from skimage.graph import MCP_Geometric

RES = 0.05
INSCRIBED = 0.1135          # footprint 0.25 x 0.227 m
V = 0.22                    # desired_linear_vel
W = 0.6                     # rotate_to_heading_angular_vel
ROT_MIN = 0.5               # rotate_to_heading_min_angle
LIDAR_RANGE = 12.0
N_RAYS = 500
SCAN_EVERY = 1              # cells of travel between scans (0.05 m; the real lidar runs at 10 Hz)
NAV_REPLAN_S = 1.0
FAIL_PENALTY_S = 20.0       # Nav2 recoveries before it aborts an impossible goal
GOAL_TOL = 0.15
NAVFN_TOL = 0.25
MAX_T = 3600.0

UNKNOWN, FREE, OCC = -1, 0, 1
N4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def load_gt(pgm):
    im = np.array(Image.open(pgm))
    return im != 254            # True = solid


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Lidar:
    def __init__(self, shape):
        self.shape = shape
        n_s = int(LIDAR_RANGE / RES * 2) + 1          # half-cell steps
        d = np.arange(n_s) * 0.5
        ang = np.arange(N_RAYS) * (2 * math.pi / N_RAYS)
        self.dr = np.rint(-np.sin(ang)[:, None] * d[None, :]).astype(np.int32)
        self.dc = np.rint(np.cos(ang)[:, None] * d[None, :]).astype(np.int32)

    def scan(self, gt, belief, r, c):
        H, Wd = self.shape
        rr = r + self.dr
        cc = c + self.dc
        inb = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < Wd)
        rrc = np.clip(rr, 0, H - 1)
        ccc = np.clip(cc, 0, Wd - 1)
        blocked = ~inb | gt[rrc, ccc]
        hit_any = blocked.any(axis=1)
        first = np.where(hit_any, blocked.argmax(axis=1), blocked.shape[1])
        idx = np.arange(blocked.shape[1])[None, :]
        free = idx < first[:, None]
        fr, fc = rrc[free], ccc[free]
        keep = belief[fr, fc] != OCC
        belief[fr[keep], fc[keep]] = FREE
        rays = np.nonzero(hit_any)[0]
        hr, hc = rr[rays, first[rays]], cc[rays, first[rays]]
        ok = (hr >= 0) & (hr < H) & (hc >= 0) & (hc < Wd)
        belief[hr[ok], hc[ok]] = OCC


def clearance(belief):
    """Distance in metres from each cell to the nearest known obstacle."""
    return ndimage.distance_transform_edt(belief != OCC) * RES


def frontier_mask(belief):
    """explore_lite's definition: unknown cells with a free 4-neighbour."""
    f = belief == FREE
    adj = np.zeros_like(f)
    adj[1:, :] |= f[:-1, :]
    adj[:-1, :] |= f[1:, :]
    adj[:, 1:] |= f[:, :-1]
    adj[:, :-1] |= f[:, 1:]
    return (belief == UNKNOWN) & adj


@dataclass
class Frontier:
    cells: np.ndarray            # (k, 2) row, col
    size: int
    centroid: tuple
    min_dist: float = 0.0
    nearest: tuple = (0, 0)


def find_frontiers(belief, min_cells=10, region=None):
    """8-connected groups of frontier cells with at least min_cells cells.
    region: optional boolean mask; keep only groups touching it (4-adjacent)."""
    fm = frontier_mask(belief)
    lab, n = ndimage.label(fm, structure=np.ones((3, 3)))
    if n == 0:
        return []
    out = []
    if region is not None:
        touch = np.zeros_like(region)
        touch[1:, :] |= region[:-1, :]
        touch[:-1, :] |= region[1:, :]
        touch[:, 1:] |= region[:, :-1]
        touch[:, :-1] |= region[:, 1:]
        ok_labels = set(np.unique(lab[touch & fm]).tolist()) - {0}
    objs = ndimage.find_objects(lab)
    for i, sl in enumerate(objs, start=1):
        if region is not None and i not in ok_labels:
            continue
        sub = lab[sl] == i
        k = int(sub.sum())
        if k < min_cells:
            continue
        rr, cc = np.nonzero(sub)
        rr = rr + sl[0].start
        cc = cc + sl[1].start
        cells = np.stack([rr, cc], axis=1)
        out.append(Frontier(cells=cells, size=k, centroid=(rr.mean(), cc.mean())))
    return out


def los_free(belief, a, b):
    """True if the straight line from a to b crosses only free cells
    (the end cell b may be unknown)."""
    (r0, c0), (r1, c1) = a, b
    n = int(max(abs(r1 - r0), abs(c1 - c0))) * 2 + 1
    rs = np.rint(np.linspace(r0, r1, n)).astype(int)
    cs = np.rint(np.linspace(c0, c1, n)).astype(int)
    m = (rs != r1) | (cs != c1)
    return bool(np.all(belief[rs[m], cs[m]] == FREE))


@dataclass
class Stats:
    distance: float = 0.0
    time: float = 0.0
    goals: int = 0
    switches: int = 0          # new goal while the previous one was still active
    failures: int = 0
    end_reason: str = ''
    path: list = field(default_factory=list)
    decisions: int = 0


class Sim:
    def __init__(self, gt, start, heading, policy):
        self.gt = gt
        self.shape = gt.shape
        self.belief = np.full(self.shape, UNKNOWN, dtype=np.int8)
        self.gt_clear = ndimage.distance_transform_edt(~gt) * RES
        self.lidar = Lidar(self.shape)
        self.pos = tuple(start)
        self.start = tuple(start)
        self.heading = heading
        self.policy = policy
        self.st = Stats()
        self.goal = None
        self.path = []
        self.last_plan_t = -1e9
        self.steps_since_scan = 0
        self.scan()

    # ------------------------------------------------------------ sensing
    def scan(self):
        self.lidar.scan(self.gt, self.belief, *self.pos)
        self.belief[self.pos] = FREE
        self.clear = clearance(self.belief)

    # ------------------------------------------------------------ Nav2 stand-in
    def nav_costs(self):
        c = np.where(self.clear > INSCRIBED, 1.0, np.inf)
        return c

    def plan(self, goal):
        """NavFn with allow_unknown and a tolerance; returns a cell path or None."""
        costs = self.nav_costs()
        costs[self.pos] = 1.0
        m = MCP_Geometric(costs, fully_connected=True)
        cum, _ = m.find_costs([self.pos])
        g = tuple(int(x) for x in goal)
        if not np.isfinite(cum[g]):
            t = int(round(NAVFN_TOL / RES))
            r0, r1 = max(0, g[0] - t), min(self.shape[0], g[0] + t + 1)
            c0, c1 = max(0, g[1] - t), min(self.shape[1], g[1] + t + 1)
            best, bd = None, 1e9
            for r in range(r0, r1):
                for c in range(c0, c1):
                    d = math.hypot(r - g[0], c - g[1])
                    if d <= t and np.isfinite(cum[r, c]) and d < bd:
                        best, bd = (r, c), d
            if best is None:
                return None
            g = best
        return m.traceback(g)

    def send_goal(self, goal):
        if self.goal is not None:
            self.st.switches += 1
        self.goal = tuple(goal)
        self.st.goals += 1
        self.path = []
        self.last_plan_t = -1e9

    def nav_step(self):
        """Advance one cell toward the goal. Returns 'active', 'succeeded' or 'failed'."""
        g = self.goal
        if math.hypot(self.pos[0] - g[0], self.pos[1] - g[1]) * RES <= GOAL_TOL:
            return 'succeeded'
        need = (not self.path or self.st.time - self.last_plan_t >= NAV_REPLAN_S
                or not self.path_ok())
        if need:
            p = self.plan(g)
            self.last_plan_t = self.st.time
            if p is None:
                return 'failed'
            new = [tuple(x) for x in p[1:]]
            if not self.path:
                self.rotate_toward(new)
            self.path = new
            if not self.path:
                return 'succeeded'   # within tolerance of the planned end cell
        nxt = self.path[0]
        if self.gt_clear[nxt] <= INSCRIBED:
            # hidden obstacle: the costmap would have seen it; re-scan and replan
            self.scan()
            self.path = []
            return 'active'
        step = math.hypot(nxt[0] - self.pos[0], nxt[1] - self.pos[1]) * RES
        self.heading = math.atan2(-(nxt[0] - self.pos[0]), nxt[1] - self.pos[1])
        self.pos = nxt
        self.path.pop(0)
        self.st.distance += step
        self.st.time += step / V
        self.st.path.append(self.pos)
        self.steps_since_scan += 1
        if self.steps_since_scan >= SCAN_EVERY:
            self.steps_since_scan = 0
            self.scan()
        if not self.path:
            end_d = math.hypot(self.pos[0] - g[0], self.pos[1] - g[1]) * RES
            if end_d <= GOAL_TOL or end_d <= NAVFN_TOL + 1e-9:
                return 'succeeded'
        return 'active'

    def path_ok(self):
        look = self.path[:40]
        return all(self.clear[c] > INSCRIBED for c in look)

    def rotate_toward(self, path):
        if not path:
            return
        tgt = path[min(len(path) - 1, 6)]
        b = math.atan2(-(tgt[0] - self.pos[0]), tgt[1] - self.pos[1])
        d = abs(wrap(b - self.heading))
        if d > ROT_MIN:
            self.st.time += d / W
        self.heading = b

    # ------------------------------------------------------------ main loop
    def run(self):
        self.policy.reset(self)
        next_decision = 0.0
        while self.st.time < MAX_T:
            event = None
            if self.goal is not None:
                res = self.nav_step()
                if res != 'active':
                    event = res
                    if res == 'failed':
                        self.st.failures += 1
                        self.st.time += FAIL_PENALTY_S
                    self.goal = None
                    self.path = []
            elif self.st.time < next_decision:
                # idle: time passes until the policy's next decision
                self.st.time = next_decision
            if event is not None or self.st.time >= next_decision:
                self.st.decisions += 1
                action, nd = self.policy.decide(self, event)
                if nd is not None:
                    next_decision = self.st.time + nd
                if action == 'done':
                    self.st.end_reason = self.policy.end_reason
                    break
                if action != 'keep':
                    self.send_goal(action)
                if self.goal is None and next_decision <= self.st.time:
                    next_decision = self.st.time + 1.0
        else:
            self.st.end_reason = 'timeout'
        return self.st
