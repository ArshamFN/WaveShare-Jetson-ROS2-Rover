"""Frontier exploration planner for the rover.

Pure numpy and scipy, no ROS imports, so the explorer node and the offline
simulator (tools/explore_sim) run exactly the same decision code.

Grid convention: a 2-D array indexed [i, j] with i along +y and j along +x,
the layout of nav_msgs/OccupancyGrid data reshaped to (height, width).
Belief values: UNKNOWN (-1), FREE (0), OCC (1).

How a decision is made:
1. Reachability. Known-free cells where the robot body fits (clearance above
   the inscribed radius) form an 8-connected graph. Dijkstra from the robot
   gives the driving distance to every reachable cell.
2. Frontiers. Unknown cells next to free cells, grouped 8-connected. Groups
   shorter than min_frontier_size are ignored.
3. Viewpoint. Rays are cast back from up to view_samples frontier cells
   through known-free space, only within view_half_angle of the direction the
   free side lies in, so a grazing glance along the frontier does not count.
   Viewpoint candidates are reachable cells, taken from the first of four
   tiers that has any: cells with goal_clearance of room that see at least a
   quarter of the samples (and at least two); cells that see that much
   without the room; cells with the room that see at least one sample; and
   any reachable cell that sees one. In that tier, the cell with the shortest
   driving distance wins. A frontier with no viewpoint (for example one seen
   through a 1 cm gap between boxes) is skipped, not tried and failed.
4. Route. A frontier's cost is the driving distance to its viewpoint plus the
   turn needed to set off. With use_route the frontiers are ordered by an open
   travelling-salesman route (nearest neighbour, then 2-opt, on driving
   distances between viewpoints) and the first one is the target.
5. Commitment. The current target is kept unless the best route is shorter
   than the best route starting with it by more than keep_ratio. A goal is
   only re-sent when its viewpoint moves by resend_distance or more.
6. Giving up. Arriving at a viewpoint without learning min_new_cells around
   the frontier since the previous decision means it cannot be seen from
   there. The next attempt must use a viewpoint at least retry_spacing away;
   after viewpoint_attempts such attempts the frontier is excluded. A goal Nav2
   fails is retried failed_retries times from a different viewpoint, then the
   frontier is excluded. Everything remembered is kept in world coordinates,
   so it survives the SLAM map growing or shifting.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

UNKNOWN, FREE, OCC = -1, 0, 1
SQRT2 = math.sqrt(2.0)


def belief_from_occupancy(data, width, height, occupied_thresh=65, free_thresh=25):
    """OccupancyGrid data (0..100, -1 unknown) to a (height, width) belief grid."""
    a = np.asarray(data, dtype=np.int16).reshape(height, width)
    b = np.full(a.shape, UNKNOWN, dtype=np.int8)
    b[(a >= 0) & (a <= free_thresh)] = FREE
    b[a >= occupied_thresh] = OCC
    return b


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class GridMap:
    belief: np.ndarray            # (H, W) int8: UNKNOWN / FREE / OCC
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def shape(self):
        return self.belief.shape

    def to_cell(self, x, y):
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def to_world(self, i, j):
        return (self.origin_x + (j + 0.5) * self.resolution,
                self.origin_y + (i + 0.5) * self.resolution)

    def cells_to_world(self, cells):
        c = np.asarray(cells, dtype=float).reshape(-1, 2)
        return np.stack([self.origin_x + (c[:, 1] + 0.5) * self.resolution,
                         self.origin_y + (c[:, 0] + 0.5) * self.resolution], axis=1)

    def world_to_cells(self, xy):
        """World points to cell indices; points outside the map are dropped."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        i = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(int)
        j = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(int)
        H, W = self.shape
        ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        return np.stack([i[ok], j[ok]], axis=1)

    def inside(self, i, j):
        H, W = self.shape
        return 0 <= i < H and 0 <= j < W


@dataclass
class Params:
    inscribed_radius: float = 0.1135      # footprint 0.25 x 0.227 m
    goal_clearance: float = 0.25          # Nav2 inflation radius; viewpoints prefer this much room
    escape_radius: float = 0.25           # free cells this close to the robot count as drivable
    min_frontier_size: float = 0.5        # m of frontier (cells x resolution)
    view_radius: float = 1.5              # m, farthest a viewpoint may be from the frontier
    view_half_angle_deg: float = 60.0
    view_ray_step_deg: float = 5.0
    view_samples: int = 32                # frontier cells sampled per frontier
    speed: float = 0.22                   # m/s, converts the turn into metres
    turn_rate: float = 0.6                # rad/s
    turn_weight: float = 1.0
    use_route: bool = True
    route_max_frontiers: int = 20         # cheapest frontiers that enter the route
    keep_ratio: float = 1.0
    resend_distance: float = 0.5          # m
    min_new_cells: int = 10               # cells around the frontier that must become known
    viewpoint_attempts: int = 2           # viewpoints tried per frontier before excluding it
    retry_spacing: float = 1.0            # m between those viewpoints
    failed_retries: int = 1               # Nav2 failures tolerated per frontier
    failed_viewpoint_radius: float = 0.3  # m around a failed goal never used again


@dataclass
class Option:
    cost: float
    cells: np.ndarray                     # frontier cells (k, 2)
    viewpoint: tuple                      # cell (i, j)
    near: tuple                           # frontier cell nearest the viewpoint
    path: list                            # cells from the robot to the viewpoint
    record: int                           # index into Planner.tried, or -1


@dataclass
class Decision:
    kind: str                             # 'goal', 'keep', 'done' or 'wait'
    goal: tuple = None                    # (x, y, yaw) when kind == 'goal'
    reason: str = ''
    info: dict = field(default_factory=dict)


def grid_graph(trav):
    """8-connected graph over traversable cells: edge weight 1 or sqrt(2) cells."""
    H, W = trav.shape
    idx = np.arange(H * W).reshape(H, W)
    rows, cols, wts = [], [], []
    for di, dj, w in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, SQRT2), (1, -1, SQRT2)):
        i0, i1 = 0, H - di
        j0, j1 = max(0, -dj), W - max(0, dj)
        m = trav[i0:i1, j0:j1] & trav[i0 + di:i1 + di, j0 + dj:j1 + dj]
        rows.append(idx[i0:i1, j0:j1][m])
        cols.append(idx[i0 + di:i1 + di, j0 + dj:j1 + dj][m])
        wts.append(np.full(int(m.sum()), w))
    r = np.concatenate(rows)
    c = np.concatenate(cols)
    w = np.concatenate(wts)
    return coo_matrix((w, (r, c)), shape=(H * W, H * W)).tocsr()


def frontier_groups(belief, min_cells):
    """8-connected groups of unknown cells with a free 4-neighbour."""
    free = belief == FREE
    adj = np.zeros_like(free)
    adj[1:, :] |= free[:-1, :]
    adj[:-1, :] |= free[1:, :]
    adj[:, 1:] |= free[:, :-1]
    adj[:, :-1] |= free[:, 1:]
    fm = (belief == UNKNOWN) & adj
    lab, _ = ndimage.label(fm, structure=np.ones((3, 3)))
    out = []
    for k, sl in enumerate(ndimage.find_objects(lab), start=1):
        if sl is None:
            continue
        sub = lab[sl] == k
        if int(sub.sum()) < min_cells:
            continue
        ii, jj = np.nonzero(sub)
        out.append(np.stack([ii + sl[0].start, jj + sl[1].start], axis=1))
    return out


class Planner:
    def __init__(self, params=None):
        self.p = params or Params()
        self._angles = np.arange(0.0, 2 * math.pi, math.radians(self.p.view_ray_step_deg))
        self.reset()

    def reset(self):
        self.target = None       # dict: cells_xy, unknown_before, fails, record, viewpoint_xy
        self.excluded_xy = []    # world-point arrays of given-up frontiers
        self.tried = []          # dict(cells_xy, vps): viewpoints that taught a frontier nothing
        self.failed_vp_xy = []   # (x, y) of goals Nav2 failed to reach
        self.stats = dict(decisions=0, goals=0, excluded=0, retries_elsewhere=0, failed_retries=0)
        self.last = None         # (GridMap, options, chosen index or None) of the latest decision

    # ---------------------------------------------------------------- helpers
    def _mask_from_xy(self, gm, xy_list, dilate=2):
        m = np.zeros(gm.shape, dtype=bool)
        for xy in xy_list:
            c = gm.world_to_cells(xy)
            if len(c):
                m[c[:, 0], c[:, 1]] = True
        if dilate and m.any():
            m = ndimage.binary_dilation(m, iterations=dilate)
        return m

    def _disc_mask(self, gm, centers, radius):
        m = np.zeros(gm.shape, dtype=bool)
        H, W = gm.shape
        r = int(math.ceil(radius / gm.resolution))
        for (x, y) in centers:
            ci, cj = gm.to_cell(x, y)
            i0, i1 = max(0, ci - r), min(H, ci + r + 1)
            j0, j1 = max(0, cj - r), min(W, cj + r + 1)
            if i0 >= i1 or j0 >= j1:
                continue
            ii, jj = np.mgrid[i0:i1, j0:j1]
            m[i0:i1, j0:j1] |= np.hypot(ii - ci, jj - cj) * gm.resolution <= radius
        return m

    def visibility(self, gm, cells):
        """How many sampled frontier cells each grid cell sees, within
        view_radius and the view cone. Returns (counts, samples used)."""
        b = gm.belief
        H, W = b.shape
        n = int(self.p.view_radius / gm.resolution * 2)
        d = np.arange(1, n + 1) * 0.5                       # half-cell steps
        di = np.rint(np.sin(self._angles)[:, None] * d[None, :]).astype(np.int32)
        dj = np.rint(np.cos(self._angles)[:, None] * d[None, :]).astype(np.int32)
        if len(cells) > self.p.view_samples:
            cells = cells[np.linspace(0, len(cells) - 1, self.p.view_samples).astype(int)]
        counts = np.zeros(H * W, dtype=np.int16)
        free = b == FREE
        half = math.radians(self.p.view_half_angle_deg)
        used = 0
        for (i, j) in cells:
            i0, i1 = max(0, i - 2), min(H, i + 3)
            j0, j1 = max(0, j - 2), min(W, j + 3)
            fi, fj = np.nonzero(free[i0:i1, j0:j1])
            if len(fi) == 0:
                continue
            used += 1
            ni = (fi + i0 - i).mean()
            nj = (fj + j0 - j).mean()
            if ni == 0 and nj == 0:
                keep = np.ones(len(self._angles), dtype=bool)
            else:
                na = math.atan2(ni, nj)
                keep = np.abs((self._angles - na + math.pi) % (2 * math.pi) - math.pi) <= half
            ri = i + di[keep]
            rj = j + dj[keep]
            inb = (ri >= 0) & (ri < H) & (rj >= 0) & (rj < W)
            ric = np.clip(ri, 0, H - 1)
            rjc = np.clip(rj, 0, W - 1)
            same = (ri == i) & (rj == j)
            blocked = (~inb | (b[ric, rjc] != FREE)) & ~same
            first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), blocked.shape[1])
            ok = np.arange(blocked.shape[1])[None, :] < first[:, None]
            counts[np.unique(ric[ok] * W + rjc[ok])] += 1
        return counts.reshape(H, W), used

    @staticmethod
    def _trace(pred, start, goal, W):
        path = []
        k = goal[0] * W + goal[1]
        while k >= 0 and k != start:
            path.append((k // W, k % W))
            k = pred[k]
        path.append((start // W, start % W))
        return path[::-1]

    def _turn_cost(self, path, yaw):
        if len(path) < 2:
            return 0.0
        t = path[min(len(path) - 1, 10)]
        b = math.atan2(t[0] - path[0][0], t[1] - path[0][1])
        return self.p.turn_weight * abs(wrap(b - yaw)) / self.p.turn_rate * self.p.speed

    # ---------------------------------------------------------------- options
    def options(self, gm, pose):
        """Frontiers that have a reachable viewpoint, with their cost from the
        robot. Returns (options, context), or (None, None) if the robot is
        outside the map."""
        res = gm.resolution
        H, W = gm.shape
        ri, rj = gm.to_cell(pose[0], pose[1])
        if not gm.inside(ri, rj):
            return None, None
        clear = ndimage.distance_transform_edt(gm.belief != OCC) * res
        trav = (gm.belief == FREE) & (clear > self.p.inscribed_radius)
        near_robot = self._disc_mask(gm, [(pose[0], pose[1])], self.p.escape_radius)
        trav |= near_robot & (gm.belief == FREE)
        trav[ri, rj] = True
        G = grid_graph(trav)
        start = ri * W + rj
        D, pred = dijkstra(G, directed=False, indices=start, return_predecessors=True)
        D = D.reshape(H, W) * res
        reach = np.isfinite(D)
        if self.failed_vp_xy:
            reach &= ~self._disc_mask(gm, self.failed_vp_xy, self.p.failed_viewpoint_radius)
        roomy = reach & (clear >= self.p.goal_clearance)
        min_cells = int(math.ceil(self.p.min_frontier_size / res - 1e-9))
        excluded = self._mask_from_xy(gm, self.excluded_xy)
        tried = [self._mask_from_xy(gm, [t['cells_xy']]) for t in self.tried]
        groups = frontier_groups(gm.belief, min_cells)
        opts, n_excluded, n_no_view = [], 0, 0
        for cells in groups:
            if excluded[cells[:, 0], cells[:, 1]].mean() > 0.5:
                n_excluded += 1
                continue
            rec = next((k for k, m in enumerate(tried) if m[cells[:, 0], cells[:, 1]].any()), -1)
            if rec >= 0:
                away = ~self._disc_mask(gm, self.tried[rec]['vps'], self.p.retry_spacing)
                pool_reach, pool_roomy = reach & away, roomy & away
            else:
                pool_reach, pool_roomy = reach, roomy
            counts, n = self.visibility(gm, cells)
            vp = None
            if n:
                # seeing a real part of the frontier matters more than room around the goal
                good = counts >= max(2, n // 4)
                some = counts >= 1
                tiers = (pool_roomy & good, pool_reach & good, pool_roomy & some, pool_reach & some)
                for ok in tiers:
                    if ok.any():
                        vp = ok
                        break
            if vp is None:
                n_no_view += 1
                continue
            vi, vj = np.nonzero(vp)
            k = int(np.argmin(D[vi, vj]))
            v = (int(vi[k]), int(vj[k]))
            near = cells[int(np.argmin(np.hypot(cells[:, 0] - v[0], cells[:, 1] - v[1])))]
            path = self._trace(pred, start, v, W)
            cost = float(D[v]) + self._turn_cost(path, pose[2])
            opts.append(Option(cost, cells, v, (int(near[0]), int(near[1])), path, rec))
        info = dict(frontiers=len(groups), excluded=n_excluded, no_viewpoint=n_no_view,
                    candidates=len(opts))
        return opts, dict(info=info, G=G, W=W)

    # ---------------------------------------------------------------- route
    @staticmethod
    def _route_len(order, d0, M):
        t = d0[order[0]]
        for a, b in zip(order[:-1], order[1:]):
            t += M[a, b]
        return t

    def _solve(self, d0, M):
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
        best = self._route_len(order, d0, M)
        improved = True
        while improved:
            improved = False
            for a in range(n - 1):
                for b in range(a + 1, n):
                    cand = order[:a] + order[a:b + 1][::-1] + order[b + 1:]
                    L = self._route_len(cand, d0, M)
                    if L < best - 1e-9:
                        order, best, improved = cand, L, True
        return order, best

    @staticmethod
    def _route_matrix(opts, ctx, res):
        W = ctx['W']
        src = [o.viewpoint[0] * W + o.viewpoint[1] for o in opts]
        Dm = dijkstra(ctx['G'], directed=False, indices=src)[:, src] * res
        M = np.where(np.isfinite(Dm), Dm, 1e6)
        return np.minimum(M, M.T)

    # ---------------------------------------------------------------- decide
    def _match_target(self, gm, opts):
        if self.target is None:
            return None
        mask = self._mask_from_xy(gm, [self.target['cells_xy']])
        for k, o in enumerate(opts):
            if mask[o.cells[:, 0], o.cells[:, 1]].any():
                return k
        return None

    def _unknown_near(self, gm, cells_xy):
        area = self._mask_from_xy(gm, [cells_xy], dilate=6)
        return int((gm.belief[area] == UNKNOWN).sum())

    def _exclude(self, gm, pose, o):
        self.excluded_xy.append(gm.cells_to_world(o.cells))
        self.stats['excluded'] += 1
        self.target = None
        return self.decide(gm, pose)

    def _nothing_learned(self, gm, pose, o):
        """Standing at a viewpoint showed this frontier nothing new: try it from
        somewhere else, or give up on it after viewpoint_attempts places."""
        here = (pose[0], pose[1])
        cells_xy = gm.cells_to_world(o.cells)
        if o.record >= 0:
            rec = self.tried[o.record]
            rec['cells_xy'] = cells_xy
            rec['vps'].append(here)
        else:
            rec = dict(cells_xy=cells_xy, vps=[here])
            self.tried.append(rec)
        if len(rec['vps']) >= self.p.viewpoint_attempts:
            return self._exclude(gm, pose, o)
        self.stats['retries_elsewhere'] += 1
        self.target = None
        return self.decide(gm, pose)

    def decide(self, gm, pose, event=None, active_goal=None):
        """pose: (x, y, yaw) in the map frame.
        event: None, or 'succeeded' / 'failed' for the goal that just ended.
        active_goal: (x, y) of the goal Nav2 is working on, or None."""
        self.stats['decisions'] += 1
        opts, ctx = self.options(gm, pose)
        if opts is None:
            return Decision('wait', reason='robot_outside_map')
        cur = self._match_target(gm, opts)

        if event in ('succeeded', 'failed') and cur is not None:
            tgt = self.target
            if event == 'failed':
                tgt['fails'] += 1
                if tgt.get('viewpoint_xy') is not None:
                    self.failed_vp_xy.append(tgt['viewpoint_xy'])
                if tgt['fails'] > self.p.failed_retries:
                    return self._exclude(gm, pose, opts[cur])
                self.stats['failed_retries'] += 1
                opts, ctx = self.options(gm, pose)      # again, without the failed viewpoint
                cur = self._match_target(gm, opts)
            else:
                # unknown cells around the frontier as it was at the previous decision
                learned = tgt['unknown_before'] - self._unknown_near(gm, tgt['cells_xy'])
                if learned < self.p.min_new_cells:
                    return self._nothing_learned(gm, pose, opts[cur])

        if not opts:
            self.last = (gm, opts, None)
            return Decision('done', reason='no_reachable_frontiers', info=ctx['info'])

        n = len(opts)
        d0 = np.array([o.cost for o in opts])
        if self.p.use_route and n > 1:
            sel = [int(k) for k in np.argsort(d0, kind='stable')[:self.p.route_max_frontiers]]
            if cur is not None and cur not in sel:
                sel.append(cur)
            M = self._route_matrix([opts[k] for k in sel], ctx, gm.resolution)
            sd0 = d0[sel]
            order, L = self._solve(sd0, M)
            choice = sel[order[0]]
            if cur is not None and choice != cur:
                # keep the current target unless the best route is clearly shorter
                ic = sel.index(cur)
                rest = [k for k in order if k != ic]
                L_cur = sd0[ic] + (self._route_len(rest, M[ic], M) if rest else 0.0)
                if L_cur <= self.p.keep_ratio * L:
                    choice = cur
        else:
            choice = int(np.argmin(d0))

        o = opts[choice]
        self.last = (gm, opts, choice)
        same = cur is not None and choice == cur
        cells_xy = gm.cells_to_world(o.cells)
        if not same:
            self.target = dict(fails=0, viewpoint_xy=None)
        self.target['cells_xy'] = cells_xy
        self.target['unknown_before'] = self._unknown_near(gm, cells_xy)
        vx, vy = gm.to_world(*o.viewpoint)
        info = dict(ctx['info'], cost=round(o.cost, 2), frontier_cells=len(o.cells))
        if same and active_goal is not None and \
                math.hypot(vx - active_goal[0], vy - active_goal[1]) < self.p.resend_distance:
            return Decision('keep', info=info)
        self.target['viewpoint_xy'] = (vx, vy)
        p = o.path
        if len(p) >= 2:
            a, b = p[max(0, len(p) - 6)], p[-1]
            yaw = math.atan2(b[0] - a[0], b[1] - a[1])
        else:
            yaw = math.atan2(o.near[0] - o.viewpoint[0], o.near[1] - o.viewpoint[1])
        self.stats['goals'] += 1
        return Decision('goal', goal=(vx, vy, yaw), info=info)
