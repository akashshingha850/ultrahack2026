#!/usr/bin/env python3
"""
Dynamic path-planning & exploration animation.

A 2-D, top-down visualisation of the SAME reactive LiDAR obstacle-avoidance
logic the real drone flies in main.py → utils.open_path_explore():

  • 8 body-frame LiDAR beams (0,45,…,315°), exactly like LidarReader.get_directions.
  • Cruise forward at full speed while the path dead-ahead is open.
  • When a wall comes within `stop_dist` ahead, STOP and pick the most-open of the
    8 directions, scored  min(clear, leg_len) − reverse_pen − revisit_pen :
        reverse_pen  → avoid backtracking (body 135°–225°)
        revisit_pen  → spread out, penalise cells already visited
    with a `min_clear` hysteresis so it only commits to a genuinely open lane.
  • A visited 5 m grid spreads coverage across the arena instead of pacing a line.
  • The "camera" finds the smoke target when it is within detection range AND in
    line of sight (no wall between) → exploration ends, just like target_found.

Pure simulation — no MAVLink, no hardware. Run with system python:
    /usr/bin/python3 dynamic_path_explore_animation.py
Save a video instead of showing a window:
    /usr/bin/python3 dynamic_path_explore_animation.py --save explore.mp4
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, Wedge


# ─────────────────────────────────────────────────────────────────────────────
# Tunables — mirror utils.open_path_explore() / config.yaml defaults
# ─────────────────────────────────────────────────────────────────────────────
SPEED        = 5.0     # m/s cruise (flight.speed)
STOP_DIST    = 5.0     # replan when a wall is this close ahead (exploration.stop_dist_m)
REPLAN_DIST  = STOP_DIST
MIN_CLEAR    = STOP_DIST + 1.0   # only commit to a direction with > this clearance (hysteresis)
LEG_LEN      = 15.0    # max waypoint leg length
ACCEPT_RAD   = 2.5     # "arrived at waypoint" radius
CELL_M       = 5.0     # visited-grid resolution
LIDAR_RANGE  = 12.0    # max LiDAR range (m), beyond = open (inf)
DETECT_RANGE = 14.0    # camera smoke-detection range (m)
DT           = 0.10    # simulation timestep (s)
YAW_RATE     = math.radians(160)  # max turn rate (rad/s) while reorienting onto a leg

ARENA = (0.0, 60.0, 0.0, 45.0)    # xmin, xmax, ymin, ymax (metres)


# ─────────────────────────────────────────────────────────────────────────────
# World: rectangular obstacle blocks + outer walls
# ─────────────────────────────────────────────────────────────────────────────
# (x, y, w, h) axis-aligned blocks the LiDAR will see as walls.
OBSTACLES = [
    (12.0,  8.0, 8.0, 6.0),
    (28.0, 20.0, 10.0, 7.0),
    (44.0,  6.0, 7.0, 9.0),
    (16.0, 28.0, 9.0, 8.0),
    (40.0, 30.0, 11.0, 8.0),
    (6.0,  22.0, 5.0, 9.0),
]

START = (4.0, 6.0)          # drone start (x, y)
START_HEADING = math.radians(20)
TARGET = (52.0, 40.0)       # smoke target (x, y)


def _segments_from_world():
    """All obstacle + outer-wall edges as (x1,y1,x2,y2) segments for ray casting."""
    segs = []
    xmin, xmax, ymin, ymax = ARENA
    # outer walls
    segs += [
        (xmin, ymin, xmax, ymin), (xmax, ymin, xmax, ymax),
        (xmax, ymax, xmin, ymax), (xmin, ymax, xmin, ymin),
    ]
    for (x, y, w, h) in OBSTACLES:
        segs += [
            (x, y, x + w, y), (x + w, y, x + w, y + h),
            (x + w, y + h, x, y + h), (x, y + h, x, y),
        ]
    return np.array(segs, dtype=float)


SEGMENTS = _segments_from_world()


def _ray_cast(ox, oy, angle, max_range, segments=SEGMENTS):
    """Distance from (ox,oy) along `angle` to the nearest segment, capped at max_range."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range
    x1, y1, x2, y2 = segments[:, 0], segments[:, 1], segments[:, 2], segments[:, 3]
    sdx, sdy = x2 - x1, y2 - y1
    denom = dx * sdy - dy * sdx
    valid = np.abs(denom) > 1e-9
    t = np.full_like(denom, np.inf)
    u = np.full_like(denom, np.inf)
    rx, ry = x1 - ox, y1 - oy
    t[valid] = (rx[valid] * sdy[valid] - ry[valid] * sdx[valid]) / denom[valid]
    u[valid] = (rx[valid] * dy - ry[valid] * dx) / denom[valid]
    hit = valid & (t > 1e-6) & (u >= -1e-9) & (u <= 1 + 1e-9)
    if np.any(hit):
        best = min(best, float(np.min(t[hit])))
    return best


def lidar_directions(x, y, yaw):
    """8 body-frame beams {0,45,…,315} → clearance (m), inf = open.
    Mirrors LidarReader.get_directions(): 0°=dead ahead, 90°=right, 180°=back."""
    dirs = {}
    for body_deg in range(0, 360, 45):
        ang = yaw + math.radians(body_deg)
        d = _ray_cast(x, y, ang, LIDAR_RANGE)
        dirs[body_deg] = float("inf") if d >= LIDAR_RANGE - 1e-6 else d
    return dirs


def has_line_of_sight(x, y, tx, ty):
    """True if nothing blocks the straight line from (x,y) to the target."""
    dist = math.hypot(tx - x, ty - y)
    ang = math.atan2(ty - y, tx - x)
    return _ray_cast(x, y, ang, dist + 0.5) >= dist - 0.3


# ─────────────────────────────────────────────────────────────────────────────
# Planner — a direct port of the open_path_explore() decision core
# ─────────────────────────────────────────────────────────────────────────────
class Explorer:
    def __init__(self):
        self.x, self.y = START
        self.yaw = START_HEADING
        self.visited: dict[tuple[int, int], int] = {}
        self.wp = None          # (wp_x, wp_y)
        self.cur_head = None    # committed leg heading (rad)
        self.commit_t = 0.0     # sim time the current leg was committed
        self.t = 0.0
        self.found = False
        self.path = [(self.x, self.y)]
        self.last_dirs = lidar_directions(self.x, self.y, self.yaw)

    # -- visited grid helpers (CELL_M resolution) --
    def _mark(self):
        key = (int(round(self.x / CELL_M)), int(round(self.y / CELL_M)))
        self.visited[key] = self.visited.get(key, 0) + 1

    def _visits_ahead(self, heading, dist):
        d = min(dist if math.isfinite(dist) else LEG_LEN, LEG_LEN)
        px = self.x + d * math.cos(heading)
        py = self.y + d * math.sin(heading)
        key = (int(round(px / CELL_M)), int(round(py / CELL_M)))
        return self.visited.get(key, 0)

    def _choose(self, dirs):
        """Best ABSOLUTE heading (rad) among the 8 beams, or None if boxed in."""
        best, best_score = None, -1e9
        for body_deg, clear in dirs.items():
            if clear <= MIN_CLEAR:
                continue
            abs_head = (self.yaw + math.radians(body_deg) + math.pi) % (2 * math.pi) - math.pi
            reverse_pen = 8.0 if 135 <= body_deg <= 225 else 0.0
            revisit_pen = 4.0 * self._visits_ahead(abs_head, clear)
            score = min(clear, LEG_LEN) - reverse_pen - revisit_pen
            if score > best_score:
                best, best_score = abs_head, score
        return best

    def _new_waypoint(self, dirs):
        head = self._choose(dirs)
        boxed = head is None
        if boxed:                                  # boxed in → reverse
            head = (self.yaw + math.pi + math.pi) % (2 * math.pi) - math.pi
        body = int(round(math.degrees(head - self.yaw)) % 360)
        clear = dirs.get(body, float("inf"))
        dist = min(LEG_LEN, max(MIN_CLEAR,
                                (clear - STOP_DIST) if math.isfinite(clear) else LEG_LEN))
        self.wp = (self.x + dist * math.cos(head), self.y + dist * math.sin(head))
        self.cur_head = head
        self.commit_t = self.t
        return boxed

    def _clear_toward(self, dirs, heading):
        body = int(round(math.degrees(heading - self.yaw) / 45.0)) * 45 % 360
        return dirs.get(body, float("inf"))

    def step(self):
        """Advance one DT. Returns a dict of state for rendering."""
        if self.found:
            return self._state(reason="FOUND")

        self.t += DT
        self._mark()
        dirs = lidar_directions(self.x, self.y, self.yaw)
        self.last_dirs = dirs

        # target acquired? (in range AND line of sight) — like target_found.set()
        if (math.hypot(TARGET[0] - self.x, TARGET[1] - self.y) <= DETECT_RANGE
                and has_line_of_sight(self.x, self.y, *TARGET)):
            self.found = True
            return self._state(reason="FOUND")

        reached = (self.wp is not None and
                   math.hypot(self.wp[0] - self.x, self.wp[1] - self.y) <= ACCEPT_RAD)
        toward = self._clear_toward(dirs, self.cur_head) if self.cur_head is not None else float("inf")
        committed = (self.t - self.commit_t) >= 1.2     # commit window before a wall-replan
        wall = self.cur_head is not None and toward <= REPLAN_DIST and committed

        reason = None
        if self.wp is None:
            reason, _ = "init", self._new_waypoint(dirs)
        elif reached:
            reason, _ = "arrived", self._new_waypoint(dirs)
        elif wall:
            reason = "wall %.1fm" % toward
            boxed = self._new_waypoint(dirs)
            if boxed:
                reason = "boxed→reverse"
        else:
            reason = "cruise"

        self._drive()
        self.path.append((self.x, self.y))
        return self._state(reason=reason)

    def _drive(self):
        """Yaw toward the committed heading, then translate at SPEED along the nose."""
        if self.cur_head is None:
            return
        err = (self.cur_head - self.yaw + math.pi) % (2 * math.pi) - math.pi
        max_step = YAW_RATE * DT
        self.yaw += max(-max_step, min(max_step, err))
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        # only translate once roughly aligned (so it doesn't drive into the wall mid-turn)
        if abs(err) < math.radians(35):
            step = SPEED * DT
            nx = self.x + step * math.cos(self.yaw)
            ny = self.y + step * math.sin(self.yaw)
            # hard safety: don't let the sim walk through a block
            if _ray_cast(self.x, self.y, self.yaw, step + 0.6) > step + 0.5:
                self.x, self.y = nx, ny

    def _state(self, reason):
        return {
            "x": self.x, "y": self.y, "yaw": self.yaw,
            "dirs": self.last_dirs, "wp": self.wp, "cur_head": self.cur_head,
            "visited": dict(self.visited), "reason": reason,
            "found": self.found, "t": self.t,
        }


# ─────────────────────────────────────────────────────────────────────────────
# Animation
# ─────────────────────────────────────────────────────────────────────────────
def build_animation():
    exp = Explorer()
    xmin, xmax, ymin, ymax = ARENA

    fig, ax = plt.subplots(figsize=(11, 8.4))
    fig.patch.set_facecolor("#0e1117")
    ax.set_facecolor("#0e1117")
    ax.set_xlim(xmin - 1, xmax + 1)
    ax.set_ylim(ymin - 1, ymax + 1)
    ax.set_aspect("equal")
    ax.set_title("Dynamic LiDAR Path Planning & Exploration  (open_path_explore)",
                 color="#e6e6e6", fontsize=13, pad=12)
    ax.tick_params(colors="#666")
    for s in ax.spines.values():
        s.set_color("#444")

    # static world
    for (x, y, w, h) in OBSTACLES:
        ax.add_patch(Rectangle((x, y), w, h, facecolor="#3a3f4b",
                               edgecolor="#6b7280", linewidth=1.2, zorder=2))
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                           fill=False, edgecolor="#6b7280", linewidth=1.5, zorder=2))
    ax.plot(*START, marker="s", color="#22c55e", markersize=9, zorder=5)
    ax.annotate("START", START, textcoords="offset points", xytext=(8, -12),
                color="#22c55e", fontsize=9)
    target_dot = ax.plot(*TARGET, marker="*", color="#f97316", markersize=20, zorder=6)[0]
    ax.annotate("SMOKE", TARGET, textcoords="offset points", xytext=(10, 6),
                color="#f97316", fontsize=9)

    # dynamic artists
    visited_artists = []
    beam_lines = [ax.plot([], [], lw=1.4, zorder=3)[0] for _ in range(8)]
    beam_dots = [ax.plot([], [], marker="o", ms=4, zorder=3, color="#ef4444")[0] for _ in range(8)]
    detect_ring = Circle(START, DETECT_RANGE, fill=False, ls=":", lw=1.0,
                         edgecolor="#f97316", alpha=0.35, zorder=2)
    ax.add_patch(detect_ring)
    path_line = ax.plot([], [], color="#38bdf8", lw=2.0, alpha=0.9, zorder=4)[0]
    wp_marker = ax.plot([], [], marker="x", color="#facc15", ms=12, mew=2.5, zorder=6)[0]
    leg_line = ax.plot([], [], color="#facc15", lw=1.4, ls="--", alpha=0.8, zorder=4)[0]
    drone = ax.plot([], [], marker="o", color="#38bdf8", ms=12, zorder=7)[0]
    nose = ax.plot([], [], color="#bae6fd", lw=2.5, zorder=7)[0]
    hud = ax.text(0.012, 0.985, "", transform=ax.transAxes, va="top", ha="left",
                  color="#e6e6e6", fontsize=9.5, family="monospace",
                  bbox=dict(boxstyle="round", fc="#1f2937", ec="#374151", alpha=0.9), zorder=10)

    def beam_color(clear):
        if not math.isfinite(clear):
            return "#22c55e"            # open
        if clear <= REPLAN_DIST:
            return "#ef4444"            # blocked → triggers replan
        if clear <= MIN_CLEAR:
            return "#f59e0b"            # marginal (below hysteresis)
        return "#84cc16"               # clear

    def update(_frame):
        nonlocal visited_artists
        st = exp.step()
        x, y, yaw = st["x"], st["y"], st["yaw"]

        # visited heatmap (redraw the few new cells cheaply each frame)
        for a in visited_artists:
            a.remove()
        visited_artists = []
        for (cx, cy), n in st["visited"].items():
            visited_artists.append(ax.add_patch(Rectangle(
                (cx * CELL_M - CELL_M / 2, cy * CELL_M - CELL_M / 2), CELL_M, CELL_M,
                facecolor="#1d4ed8", alpha=min(0.07 + 0.05 * n, 0.32),
                edgecolor="none", zorder=1)))

        # 8 LiDAR beams
        for i, (body_deg, clear) in enumerate(sorted(st["dirs"].items())):
            ang = yaw + math.radians(body_deg)
            d = clear if math.isfinite(clear) else LIDAR_RANGE
            ex, ey = x + d * math.cos(ang), y + d * math.sin(ang)
            beam_lines[i].set_data([x, ex], [y, ey])
            beam_lines[i].set_color(beam_color(clear))
            beam_lines[i].set_alpha(0.55 if math.isfinite(clear) else 0.25)
            if math.isfinite(clear):
                beam_dots[i].set_data([ex], [ey])
            else:
                beam_dots[i].set_data([], [])

        # path / waypoint / leg
        px, py = zip(*exp.path)
        path_line.set_data(px, py)
        if st["wp"] is not None:
            wp_marker.set_data([st["wp"][0]], [st["wp"][1]])
            leg_line.set_data([x, st["wp"][0]], [y, st["wp"][1]])
        detect_ring.center = (x, y)

        # drone body + nose
        drone.set_data([x], [y])
        nose.set_data([x, x + 2.6 * math.cos(yaw)], [y, y + 2.6 * math.sin(yaw)])

        # found → flash target, freeze
        if st["found"]:
            target_dot.set_markersize(28)
            detect_ring.set_edgecolor("#22c55e")
            detect_ring.set_alpha(0.6)

        d8 = "  ".join(f"{a}:{'inf' if not math.isfinite(c) else f'{c:4.1f}'}"
                       for a, c in sorted(st["dirs"].items()))
        # hud.set_text(
        #     f" t={st['t']:5.1f}s   pos=({x:5.1f},{y:5.1f})   yaw={math.degrees(yaw)%360:5.1f}°\n"
        #     f" state: {st['reason']:<14} cells={len(st['visited'])}\n"
        #     f" 8-dir(body, m): {d8}\n"
        #     f" {'>>> TARGET ACQUIRED — exploration ends <<<' if st['found'] else ''}"
        # )
        return []

    frames = 1200
    anim = FuncAnimation(fig, update, frames=frames, interval=DT * 1000,
                         blit=False, repeat=False)
    return fig, anim


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save", metavar="FILE",
                    help="write the animation to a video/gif instead of showing a window")
    ap.add_argument("--fps", type=int, default=int(round(1 / DT)),
                    help="frames per second when saving (default %(default)s)")
    args = ap.parse_args()

    fig, anim = build_animation()
    if args.save:
        print(f"Rendering → {args.save} …")
        anim.save(args.save, fps=args.fps, dpi=120)
        print("done.")
    else:
        plt.show()


if __name__ == "__main__":
    main()
