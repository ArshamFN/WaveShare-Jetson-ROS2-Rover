"""Exploration policies for sim.py.

decide(sim, event) -> (action, next_interval_s)
  action: 'done', 'keep' (no new goal now) or a goal cell (row, col)
  event:  None (periodic tick), 'succeeded' or 'failed' (the last goal ended)
"""
import math

import numpy as np
from scipy import ndimage
from skimage.graph import MCP_Geometric

from sim import FREE, INSCRIBED, RES, V, W, find_frontiers, los_free, wrap


def frontier_distances(sim, fr):
    r, c = sim.pos
    for f in fr:
        d = np.hypot(f.cells[:, 0] - r, f.cells[:, 1] - c)
        i = int(d.argmin())
        f.min_dist = float(d[i]) * RES
        f.nearest = (int(f.cells[i, 0]), int(f.cells[i, 1]))
    return fr


class ExploreLite:
    """m-explore-ros2 at 326cf8a on Humble: straight-line cost
    potential*min_distance - gain*size, goal at the frontier centroid,
    replan at planner_frequency and right after a goal succeeds, blacklist on
    abort (then wait for the next timer tick) or after progress_timeout."""
    name = 'A explore_lite'

    def __init__(self, potential=3.0, gain=1.0, freq=0.15, progress_timeout=30.0, min_cells=10, name=None):
        self.potential, self.gain, self.freq = potential, gain, freq
        self.progress_timeout, self.min_cells = progress_timeout, min_cells
        if name:
            self.name = name

    def reset(self, sim):
        self.blacklist = []
        self.prev_goal = None
        self.prev_distance = math.inf
        self.last_progress = 0.0
        self.target = None
        self.end_reason = ''

    def frontiers(self, sim):
        free = sim.belief == FREE
        lab, _ = ndimage.label(free)          # 4-connected, like its BFS
        region = lab == lab[sim.pos]
        return frontier_distances(sim, find_frontiers(sim.belief, self.min_cells, region))

    def on_blacklist(self, p):
        tol = 5  # cells, explore.cpp goalOnBlacklist
        return any(abs(p[0] - b[0]) < tol and abs(p[1] - b[1]) < tol for b in self.blacklist)

    def select(self, sim, cand):
        return min(cand, key=lambda f: self.potential * f.min_dist - self.gain * f.size * RES)

    def decide(self, sim, event):
        interval = 1.0 / self.freq
        if event == 'failed':
            self.blacklist.append(self.target)
            return 'keep', interval / 2      # waits for its next timer tick
        if event == 'succeeded':
            self.last_progress = sim.st.time
            self.prev_distance = 0.0
        return self.make_plan(sim, interval)

    def make_plan(self, sim, interval):
        fr = self.frontiers(sim)
        if not fr:
            self.end_reason = 'no_frontiers'
            return 'done', None
        cand = [f for f in fr if not self.on_blacklist(f.centroid)]
        if not cand:
            self.end_reason = 'all_blacklisted'
            return 'done', None
        f = self.select(sim, cand)
        target = f.centroid
        same = self.prev_goal is not None and math.hypot(
            target[0] - self.prev_goal[0], target[1] - self.prev_goal[1]) * RES < 0.01
        self.prev_goal = target
        if not same or self.prev_distance > f.min_dist:
            self.last_progress = sim.st.time
            self.prev_distance = f.min_dist
        active = sim.goal is not None
        if active and sim.st.time - self.last_progress > self.progress_timeout:
            self.blacklist.append(target)
            return self.make_plan(sim, interval)
        if same and active:
            return 'keep', interval
        self.target = target
        return (int(round(target[0])), int(round(target[1]))), interval


class Sector(ExploreLite):
    """Arsham's rule on explore_lite's machinery: if a frontier cell lies within
    2 m, within +-35 deg of the heading and in line of sight, go to the closest
    such frontier; otherwise go to the nearest frontier (straight-line)."""
    name = 'B sector rule'

    def __init__(self, radius=2.0, half_angle_deg=35.0, **kw):
        super().__init__(**kw)
        self.radius, self.half = radius, math.radians(half_angle_deg)

    def select(self, sim, cand):
        r, c = sim.pos
        best = None
        for f in cand:
            dr = f.cells[:, 0] - r
            dc = f.cells[:, 1] - c
            d = np.hypot(dr, dc) * RES
            ang = np.arctan2(-dr, dc)
            dang = np.abs((ang - sim.heading + np.pi) % (2 * np.pi) - np.pi)
            idx = np.nonzero((d <= self.radius) & (dang <= self.half))[0]
            for i in idx[np.argsort(d[idx])][:8]:
                if los_free(sim.belief, sim.pos, tuple(f.cells[i])):
                    if best is None or d[i] < best[0]:
                        best = (d[i], f)
                    break
        if best is not None:
            return best[1]
        return min(cand, key=lambda f: f.min_dist)


class PathFrontier:
    """C: reachability on the inflated known-free map, path distance,
    an approach point with line of sight to the frontier, a turn penalty,
    and commitment to the current target."""
    name = 'C path+reach+commit'

    def __init__(self, interval=2.0, r_obs=1.5, w_turn=1.0, w_gain=0.0,
                 switch_ratio=0.6, switch_margin=1.0, min_cells=10, view_half_deg=60.0,
                 approach_fd=0.0, retry_closer=True, name=None):
        self.interval, self.r_obs, self.w_turn, self.w_gain = interval, r_obs, w_turn, w_gain
        self.view_half = math.radians(view_half_deg)
        self.approach_fd = approach_fd
        self.retry_closer = retry_closer
        self.switch_ratio, self.switch_margin, self.min_cells = switch_ratio, switch_margin, min_cells
        if name:
            self.name = name

    def reset(self, sim):
        self.target = None          # (cells set, approach cell)
        self.excluded = np.zeros(sim.shape, dtype=bool)
        self.closer = np.zeros(sim.shape, dtype=bool)
        self.end_reason = ''

    # -------------------------------------------------------------- helpers
    def reach(self, sim):
        trav = (sim.belief == FREE) & (sim.clear > INSCRIBED)
        costs = np.where(trav, 1.0, np.inf)
        costs[sim.pos] = 1.0
        m = MCP_Geometric(costs, fully_connected=True)
        D, _ = m.find_costs([sim.pos])
        return trav, m, D * RES

    # rays cast backwards from frontier cells: a cell is a valid approach point
    # only if a straight line of known-free cells joins it to the frontier
    _ANG = np.arange(0, 2 * math.pi, math.radians(5))

    def visible_from(self, sim, f):
        H, Wd = sim.shape
        n = int(self.r_obs / RES * 2)
        d = np.arange(1, n + 1) * 0.5
        dr = np.rint(-np.sin(self._ANG)[:, None] * d[None, :]).astype(np.int32)
        dc = np.rint(np.cos(self._ANG)[:, None] * d[None, :]).astype(np.int32)
        cells = f.cells
        if len(cells) > 32:
            cells = cells[np.linspace(0, len(cells) - 1, 32).astype(int)]
        count = np.zeros(sim.shape, dtype=np.int16)
        free = sim.belief == FREE
        for (r, c) in cells:
            # direction from this frontier cell toward the free side; only look back
            # within +-view_half of it, so a grazing view along the frontier line
            # (which reveals nothing behind it) does not count
            w0, w1 = max(0, r - 2), min(H, r + 3)
            v0, v1 = max(0, c - 2), min(Wd, c + 3)
            fr_, fc_ = np.nonzero(free[w0:w1, v0:v1])
            if len(fr_) == 0:
                continue
            nr = (fr_ + w0 - r).mean()
            nc = (fc_ + v0 - c).mean()
            if nr == 0 and nc == 0:
                keep_rays = slice(None)
            else:
                na = math.atan2(-nr, nc)
                keep_rays = np.abs((self._ANG - na + np.pi) % (2 * np.pi) - np.pi) <= self.view_half
            rr = r + dr[keep_rays]
            cc = c + dc[keep_rays]
            inb = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < Wd)
            rrc = np.clip(rr, 0, H - 1)
            ccc = np.clip(cc, 0, Wd - 1)
            same = (rr == r) & (cc == c)
            blocked = (~inb | (sim.belief[rrc, ccc] != FREE)) & ~same
            first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), blocked.shape[1])
            ok = np.arange(blocked.shape[1])[None, :] < first[:, None]
            one = np.zeros(sim.shape, dtype=bool)
            one[rrc[ok], ccc[ok]] = True
            count += one
        return count, len(cells)

    def approach(self, sim, f, trav, D):
        """Reachable cell that sees a good part of the frontier (at least a
        quarter of the sampled cells, so a grazing glimpse past a corner does
        not count) and minimises path length to it plus the remaining
        straight line to the frontier. Returns (cell, score)."""
        count, n = self.visible_from(sim, f)
        reach = trav & np.isfinite(D)
        ok = reach & (count >= max(2, n // 4))
        if not ok.any():
            ok = reach & (count >= 1)
        rr, cc = np.nonzero(ok)
        if len(rr) == 0:
            return None
        fd = np.min(np.hypot(rr[:, None] - f.cells[None, :, 0], cc[:, None] - f.cells[None, :, 1]), axis=1) * RES \
            if len(rr) * len(f.cells) < 4_000_000 else np.zeros(len(rr))
        w_fd = 1.0 if self.closer[f.cells[:, 0], f.cells[:, 1]].mean() > 0.5 else self.approach_fd
        score = D[rr, cc] + w_fd * fd
        k = int(np.argmin(score))
        return (int(rr[k]), int(cc[k])), float(score[k])

    def turn_cost(self, sim, m, a):
        p = m.traceback(a)
        if len(p) < 2:
            return 0.0
        t = p[min(len(p) - 1, 10)]
        b = math.atan2(-(t[0] - sim.pos[0]), t[1] - sim.pos[1])
        return self.w_turn * abs(wrap(b - sim.heading)) / W * V

    def options(self, sim):
        trav, m, D = self.reach(sim)
        out = []
        for f in find_frontiers(sim.belief, self.min_cells):
            if self.excluded[f.cells[:, 0], f.cells[:, 1]].mean() > 0.5:
                continue
            ap = self.approach(sim, f, trav, D)
            if ap is None:
                continue
            a, score = ap
            cost = score + self.turn_cost(sim, m, a) - self.w_gain * min(f.size * RES, 2.0)
            out.append((cost, f, a))
        return out

    @staticmethod
    def dilated_set(cells, shape, k=2):
        m = np.zeros(shape, dtype=bool)
        m[cells[:, 0], cells[:, 1]] = True
        return ndimage.binary_dilation(m, iterations=k)

    # -------------------------------------------------------------- decide
    def decide(self, sim, event):
        opts = self.options(sim)
        if not opts:
            self.end_reason = 'no_reachable_frontiers'
            return 'done', None
        best = min(opts, key=lambda o: o[0])
        cur = None
        if self.target is not None:
            mask = self.target[0]
            for o in opts:
                if mask[o[1].cells[:, 0], o[1].cells[:, 1]].any():
                    cur = o
                    break
        if event in ('succeeded', 'failed') and cur is not None:
            # the robot stood at its approach point (or Nav2 gave up) and the frontier barely changed:
            # it cannot be observed from there, so stop chasing it
            area, unknown_before = self.target[3], self.target[4]
            resolved = unknown_before - int((sim.belief[area] == -1).sum())
            if event == 'failed' or resolved < 10:
                closer = self.closer[cur[1].cells[:, 0], cur[1].cells[:, 1]].mean() > 0.5
                if event == 'failed' or closer or not self.retry_closer:
                    self.excluded |= self.dilated_set(cur[1].cells, sim.shape)
                else:
                    # seen from where it first became visible and nothing new: try once from right next to it
                    self.closer |= self.dilated_set(cur[1].cells, sim.shape)
                self.target = None
                return self.decide(sim, None)
        if cur is None:
            choice = best
        else:
            switch = (best is not cur and best[0] < self.switch_ratio * cur[0]
                      and cur[0] - best[0] > self.switch_margin)
            choice = best if switch else cur
        cost, f, a = choice
        area = self.dilated_set(f.cells, sim.shape, 6)
        self.target = (self.dilated_set(f.cells, sim.shape), a, f.cells.copy(), area,
                       int((sim.belief[area] == -1).sum()))
        if sim.goal is not None and math.hypot(a[0] - sim.goal[0], a[1] - sim.goal[1]) * RES < 0.5 \
                and choice is cur:
            return 'keep', self.interval
        return a, self.interval


class PathFrontierTour(PathFrontier):
    """D: C plus a global tour: order all reachable frontiers by an open
    travelling-salesman path from the robot (nearest neighbour + 2-opt on
    path distances between approach points) and head for the first one."""
    name = 'D path+reach+tour'

    def __init__(self, keep_ratio=1.1, **kw):
        kw.setdefault('approach_fd', 0.0)
        super().__init__(**kw)
        self.keep_ratio = keep_ratio

    @staticmethod
    def tour_len(order, d0, M):
        if not order:
            return 0.0
        t = d0[order[0]]
        for i in range(len(order) - 1):
            t += M[order[i], order[i + 1]]
        return t

    def solve(self, d0, M):
        n = len(d0)
        left = set(range(n))
        cur = int(np.argmin(d0))
        order = [cur]
        left.remove(cur)
        while left:
            nxt = min(left, key=lambda j: M[cur, j])
            order.append(nxt)
            left.remove(nxt)
            cur = nxt
        best = self.tour_len(order, d0, M)
        improved = True
        while improved:
            improved = False
            for i in range(len(order) - 1):
                for j in range(i + 1, len(order)):
                    cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                    L = self.tour_len(cand, d0, M)
                    if L < best - 1e-9:
                        order, best, improved = cand, L, True
        return order, best

    def decide(self, sim, event):
        opts = self.options(sim)
        if not opts:
            self.end_reason = 'no_reachable_frontiers'
            return 'done', None
        cur = None
        if self.target is not None:
            mask = self.target[0]
            for o in opts:
                if mask[o[1].cells[:, 0], o[1].cells[:, 1]].any():
                    cur = o
                    break
        if event in ('succeeded', 'failed') and cur is not None:
            area, unknown_before = self.target[3], self.target[4]
            resolved = unknown_before - int((sim.belief[area] == -1).sum())
            if event == 'failed' or resolved < 10:
                self.excluded |= self.dilated_set(cur[1].cells, sim.shape)
                self.target = None
                return self.decide(sim, None)
        n = len(opts)
        d0 = np.array([o[0] for o in opts])
        M = np.zeros((n, n))
        if n > 1:
            trav = (sim.belief == FREE) & (sim.clear > INSCRIBED)
            costs = np.where(trav, 1.0, np.inf)
            for i, o in enumerate(opts):
                c = costs.copy()
                c[o[2]] = 1.0
                Di, _ = MCP_Geometric(c, fully_connected=True).find_costs([o[2]])
                for j, o2 in enumerate(opts):
                    M[i, j] = Di[o2[2]] * RES if np.isfinite(Di[o2[2]]) else 1e6
            M = np.minimum(M, M.T)
        order, L = self.solve(d0, M)
        choice = opts[order[0]]
        if cur is not None and choice is not cur:
            # keep the current target unless the best tour is clearly shorter than one starting with it
            i_cur = next(i for i, o in enumerate(opts) if o is cur)
            rest = [k for k in order if k != i_cur]
            L_cur = d0[i_cur] + (self.tour_len(rest, M[i_cur], M) if rest else 0.0)
            if L_cur <= self.keep_ratio * L:
                choice = cur
        cost, f, a = choice
        area = self.dilated_set(f.cells, sim.shape, 6)
        self.target = (self.dilated_set(f.cells, sim.shape), a, f.cells.copy(), area,
                       int((sim.belief[area] == -1).sum()))
        if sim.goal is not None and choice is cur and \
                math.hypot(a[0] - sim.goal[0], a[1] - sim.goal[1]) * RES < 0.5:
            return 'keep', self.interval
        return a, self.interval
