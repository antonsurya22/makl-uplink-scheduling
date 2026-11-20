#!/usr/bin/env python3

import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


class SpatialAdapter:
    """
    B2 static spatial adapter.

    Two traversability representations are intentionally separated:

    1. safe_free:
       static geometry inflated by SAFETY_CLEARANCE = 0.45 m.
       Used for validating the reference route.

    2. progress_free:
       static geometry inflated by SAFETY_CLEARANCE = 0.45 m.
       Used for geodesic progress measurement so reward cannot
       prefer a geometrically shorter but safety-infeasible shortcut.

    Progress deliberately does NOT use Nav2 distance_remaining.
    """

    ROBOT_RADIUS = 0.14
    SAFETY_CLEARANCE = 0.45

    def __init__(
        self,
        root,
        name,
        progress_clearance=SAFETY_CLEARANCE,
        safety_clearance=SAFETY_CLEARANCE,
    ):
        self.root = Path(root).expanduser().resolve()
        self.name = str(name)

        self.progress_clearance = float(progress_clearance)
        self.safety_clearance = float(safety_clearance)

        self.meta_path = self.root / f"{self.name}.json"
        self.yaml_path = self.root / f"{self.name}.yaml"
        self.pgm_path = self.root / f"{self.name}.pgm"
        self.world_path = self.root / f"{self.name}.world"

        for p in (
            self.meta_path,
            self.yaml_path,
            self.pgm_path,
            self.world_path,
        ):
            if not p.exists():
                raise FileNotFoundError(p)

        self.meta = json.loads(
            self.meta_path.read_text()
        )

        self.start = tuple(
            float(v)
            for v in self.meta["start"]
        )

        self.goal = tuple(
            float(v)
            for v in self.meta["goal"]
        )

        self.resolution, self.origin = (
            self._read_yaml()
        )

        img = np.asarray(
            Image.open(self.pgm_path),
            dtype=np.uint8,
        )

        # worldgen.write_map():
        #   occupied -> 0
        #   free     -> 254
        # and then np.flipud() before writing.
        #
        # Convert back into world/map grid convention:
        # grid[j, i], j increasing with world +y.
        self.occupied = np.flipud(
            img < 128
        )

        self.progress_blocked = self._inflate(
            self.occupied,
            self.progress_clearance,
        )

        self.safe_blocked = self._inflate(
            self.occupied,
            self.safety_clearance,
        )

        self.progress_free = ~self.progress_blocked
        self.safe_free = ~self.safe_blocked

        self.goal_cell = self.world_to_cell(
            self.goal[0],
            self.goal[1],
        )

        self.start_cell = self.world_to_cell(
            self.start[0],
            self.start[1],
        )

        if not self._inside(*self.start_cell):
            raise RuntimeError(
                f"{self.name}: start outside map"
            )

        if not self._inside(*self.goal_cell):
            raise RuntimeError(
                f"{self.name}: goal outside map"
            )

        # Geodesic distance to FINAL GOAL.
        self.progress_distance = self._dijkstra_from_goal(
            self.progress_free,
            self.goal_cell,
        )

        self.safe_distance = self._dijkstra_from_goal(
            self.safe_free,
            self.goal_cell,
        )

        sj, si = self.start_cell

        if not math.isfinite(
            float(self.progress_distance[sj, si])
        ):
            raise RuntimeError(
                f"{self.name}: start-goal disconnected "
                "on progress grid"
            )

        if not math.isfinite(
            float(self.safe_distance[sj, si])
        ):
            raise RuntimeError(
                f"{self.name}: start-goal disconnected "
                "on 0.45 m safety grid"
            )

        self.initial_progress_distance = float(
            self.progress_distance[sj, si]
        )

        self.initial_safe_distance = float(
            self.safe_distance[sj, si]
        )

        self.reference_cells = (
            self._trace_reference_path()
        )

        self.reference_path = [
            self.cell_to_world(j, i)
            for j, i in self.reference_cells
        ]

    # --------------------------------------------------------------
    # File parsing
    # --------------------------------------------------------------

    def _read_yaml(self):
        vals = {}

        for line in self.yaml_path.read_text().splitlines():
            line = line.strip()

            if (
                not line
                or line.startswith("#")
                or ":" not in line
            ):
                continue

            key, value = line.split(":", 1)
            vals[key.strip()] = value.strip()

        res = float(vals["resolution"])

        origin = tuple(
            float(v.strip())
            for v in vals["origin"]
            .strip("[]")
            .split(",")
        )

        return res, origin

    # --------------------------------------------------------------
    # Geometry
    # --------------------------------------------------------------

    def world_to_cell(self, x, y):
        ox, oy, _ = self.origin

        i = int(
            math.floor(
                (float(x) - ox)
                / self.resolution
            )
        )

        j = int(
            math.floor(
                (float(y) - oy)
                / self.resolution
            )
        )

        return j, i

    def cell_to_world(self, j, i):
        ox, oy, _ = self.origin

        x = (
            ox
            + (float(i) + 0.5)
            * self.resolution
        )

        y = (
            oy
            + (float(j) + 0.5)
            * self.resolution
        )

        return x, y

    def _inside(self, j, i):
        h, w = self.occupied.shape

        return (
            0 <= j < h
            and 0 <= i < w
        )

    # --------------------------------------------------------------
    # Inflation
    # --------------------------------------------------------------

    def _inflate(self, occupied, radius):
        r = int(
            math.ceil(
                float(radius)
                / self.resolution
            )
        )

        if r <= 0:
            return occupied.copy()

        h, w = occupied.shape
        out = occupied.copy()

        # Explicit slicing: no np.roll wrap-around.
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):

                if (
                    di * di + dj * dj
                    > r * r
                ):
                    continue

                src_j0 = max(0, -dj)
                src_j1 = min(h, h - dj)

                src_i0 = max(0, -di)
                src_i1 = min(w, w - di)

                dst_j0 = src_j0 + dj
                dst_j1 = src_j1 + dj

                dst_i0 = src_i0 + di
                dst_i1 = src_i1 + di

                if (
                    src_j1 <= src_j0
                    or src_i1 <= src_i0
                ):
                    continue

                out[
                    dst_j0:dst_j1,
                    dst_i0:dst_i1,
                ] |= occupied[
                    src_j0:src_j1,
                    src_i0:src_i1,
                ]

        # Outside-map space is considered unavailable.
        out[:r, :] = True
        out[-r:, :] = True
        out[:, :r] = True
        out[:, -r:] = True

        return out

    # --------------------------------------------------------------
    # Geodesic distance
    # --------------------------------------------------------------

    @staticmethod
    def _neighbor_specs():
        root2 = math.sqrt(2.0)

        return (
            (-1,  0, 1.0),
            (+1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0, +1, 1.0),
            (-1, -1, root2),
            (-1, +1, root2),
            (+1, -1, root2),
            (+1, +1, root2),
        )

    def _valid_neighbors(self, free, j, i):
        h, w = free.shape

        for dj, di, factor in self._neighbor_specs():
            nj = j + dj
            ni = i + di

            if not (
                0 <= nj < h
                and 0 <= ni < w
            ):
                continue

            if not free[nj, ni]:
                continue

            # Do not cut diagonally through occupied corners.
            if dj != 0 and di != 0:
                if not (
                    free[j, ni]
                    and free[nj, i]
                ):
                    continue

            yield (
                nj,
                ni,
                factor * self.resolution,
            )

    def _dijkstra_from_goal(
        self,
        free,
        goal_cell,
    ):
        h, w = free.shape

        dist = np.full(
            (h, w),
            np.inf,
            dtype=np.float64,
        )

        gj, gi = goal_cell

        if not (
            self._inside(gj, gi)
            and free[gj, gi]
        ):
            raise RuntimeError(
                f"{self.name}: goal blocked for "
                "requested clearance"
            )

        dist[gj, gi] = 0.0

        heap = [
            (0.0, gj, gi)
        ]

        while heap:
            d, j, i = heapq.heappop(heap)

            if d > dist[j, i] + 1e-12:
                continue

            for nj, ni, cost in (
                self._valid_neighbors(
                    free,
                    j,
                    i,
                )
            ):
                nd = d + cost

                if nd + 1e-12 < dist[nj, ni]:
                    dist[nj, ni] = nd

                    heapq.heappush(
                        heap,
                        (nd, nj, ni),
                    )

        return dist

    # --------------------------------------------------------------
    # Runtime progress lookup
    # --------------------------------------------------------------

    def _nearest_finite_cell(
        self,
        distance_map,
        j,
        i,
        max_snap_m=0.20,
    ):
        if (
            self._inside(j, i)
            and math.isfinite(
                float(distance_map[j, i])
            )
        ):
            return j, i, 0.0

        max_r = int(
            math.ceil(
                float(max_snap_m)
                / self.resolution
            )
        )

        best = None

        for rj in range(
            max(0, j - max_r),
            min(distance_map.shape[0], j + max_r + 1),
        ):
            for ri in range(
                max(0, i - max_r),
                min(distance_map.shape[1], i + max_r + 1),
            ):
                if not math.isfinite(
                    float(distance_map[rj, ri])
                ):
                    continue

                error = (
                    math.hypot(
                        rj - j,
                        ri - i,
                    )
                    * self.resolution
                )

                if (
                    error
                    > max_snap_m + 1e-9
                ):
                    continue

                if (
                    best is None
                    or error < best[2]
                ):
                    best = (
                        rj,
                        ri,
                        error,
                    )

        if best is None:
            raise RuntimeError(
                f"{self.name}: no valid geodesic cell "
                f"within {max_snap_m:.2f} m"
            )

        return best

    def _ensure_reference_progress_cache(self):
        """
        Build the continuous progress coordinate associated with the
        frozen 0.45-m safety-feasible reference path.

        The reference path itself is generated from the validated
        safety geodesic field. Only the continuous position-to-progress
        mapping is changed here.
        """

        if hasattr(
            self,
            "_reference_progress_points",
        ):
            return

        points = [
            self.cell_to_world(*cell)
            for cell in self.reference_cells
        ]

        if len(points) < 2:
            raise RuntimeError(
                "reference path contains fewer than two points"
            )

        segment_lengths = []

        for k in range(
            len(points) - 1
        ):

            x0, y0 = points[k]
            x1, y1 = points[k + 1]

            length = math.hypot(
                x1 - x0,
                y1 - y0,
            )

            if length <= 1e-12:
                raise RuntimeError(
                    "zero-length reference-path segment"
                )

            segment_lengths.append(
                float(length)
            )

        remaining = [
            0.0
        ] * len(points)

        for k in range(
            len(points) - 2,
            -1,
            -1,
        ):

            remaining[k] = (
                remaining[k + 1]
                + segment_lengths[k]
            )

        # The reference polyline must preserve exactly the validated
        # safety-geodesic start-to-goal distance.
        reference_total = float(
            remaining[0]
        )

        safe_total = float(
            self.initial_safe_distance
        )

        if abs(
            reference_total
            - safe_total
        ) > 1e-6:

            raise RuntimeError(
                "reference progress length differs from "
                "validated safety geodesic: "
                f"{reference_total:.9f} vs "
                f"{safe_total:.9f}"
            )

        self._reference_progress_points = tuple(
            (
                float(x),
                float(y),
            )
            for x, y in points
        )

        self._reference_progress_lengths = tuple(
            segment_lengths
        )

        self._reference_progress_remaining = tuple(
            float(v)
            for v in remaining
        )


    def reference_path_distance(
        self,
        x,
        y,
    ):
        """
        Continuous remaining distance to the final goal.

        The robot is projected onto the geometrically nearest segment
        of the frozen safety-feasible reference path.

        distance =
            remaining reference-path arc length
            + lateral distance to the projected segment.

        Returns:
            (distance, lateral_error, segment_index)
        """

        self._ensure_reference_progress_cache()

        qx = float(x)
        qy = float(y)

        points = (
            self._reference_progress_points
        )

        lengths = (
            self._reference_progress_lengths
        )

        remaining = (
            self._reference_progress_remaining
        )

        best = None

        for k, length in enumerate(
            lengths
        ):

            x0, y0 = points[k]
            x1, y1 = points[k + 1]

            vx = (
                x1 - x0
            )

            vy = (
                y1 - y0
            )

            length_sq = (
                length * length
            )

            t = (
                (
                    (qx - x0) * vx
                    + (qy - y0) * vy
                )
                / length_sq
            )

            t = max(
                0.0,
                min(
                    1.0,
                    float(t),
                ),
            )

            px = (
                x0
                + t * vx
            )

            py = (
                y0
                + t * vy
            )

            lateral = math.hypot(
                qx - px,
                qy - py,
            )

            along_remaining = (
                remaining[k]
                - t * length
            )

            value = (
                float(along_remaining)
                + float(lateral)
            )

            # Geometric proximity is the primary selector.
            # Remaining distance only breaks an exact numerical tie.
            candidate = (
                float(lateral),
                float(value),
                int(k),
            )

            if (
                best is None
                or candidate < best
            ):
                best = candidate

        if best is None:
            raise RuntimeError(
                "no valid safety-reference path segment"
            )

        lateral, value, segment_index = (
            best
        )

        return (
            max(
                0.0,
                float(value),
            ),
            float(lateral),
            int(segment_index),
        )


    def _ensure_sparse_progress_cache(self):
        """
        Build the sparse safety-feasible route used for B2 runtime
        progress.

        Route:
            START -> P1 -> P2 -> ... -> final goal

        navigation_waypoints() is itself derived from the validated
        safety-reference path.
        """

        if hasattr(
            self,
            "_sparse_progress_points",
        ):
            return

        sx, sy, _ = self.start

        waypoints = list(
            self.navigation_waypoints()
        )

        if not waypoints:
            raise RuntimeError(
                "navigation_waypoints() returned no route"
            )

        points = [
            (
                float(sx),
                float(sy),
            )
        ]

        names = []

        for (
            name,
            x,
            y,
            yaw,
        ) in waypoints:

            names.append(
                str(name)
            )

            points.append(
                (
                    float(x),
                    float(y),
                )
            )

        lengths = []

        for k in range(
            len(points) - 1
        ):

            x0, y0 = points[k]
            x1, y1 = points[k + 1]

            length = math.hypot(
                x1 - x0,
                y1 - y0,
            )

            if length <= 1e-12:
                raise RuntimeError(
                    f"zero-length sparse route leg {k}"
                )

            lengths.append(
                float(length)
            )

        # tail[k] =
        # distance from the END of active leg k through all
        # subsequent sparse legs to the final goal.
        tails = []

        for k in range(
            len(lengths)
        ):

            tails.append(
                float(
                    sum(
                        lengths[k + 1:]
                    )
                )
            )

        self._sparse_progress_points = tuple(
            points
        )

        self._sparse_progress_names = tuple(
            names
        )

        self._sparse_progress_lengths = tuple(
            lengths
        )

        self._sparse_progress_tails = tuple(
            tails
        )

        self._initial_sparse_route_distance = float(
            sum(lengths)
        )


    @property
    def initial_sparse_route_distance(self):
        """
        Total START-to-goal length of the sparse safety-feasible route.
        """

        self._ensure_sparse_progress_cache()

        return float(
            self._initial_sparse_route_distance
        )


    def active_sparse_leg_distance(
        self,
        x,
        y,
        leg_index,
    ):
        """
        Continuous remaining distance using only the currently active
        sparse Nav2 route leg.

        remaining =
            remaining distance on active leg
            + future sparse-leg tail
            + lateral error to active leg

        Returns:
            remaining_distance,
            lateral_error,
            leg_fraction
        """

        self._ensure_sparse_progress_cache()

        leg = int(
            leg_index
        )

        lengths = (
            self._sparse_progress_lengths
        )

        if (
            leg < 0
            or leg >= len(lengths)
        ):
            raise IndexError(
                f"invalid sparse leg index {leg}; "
                f"valid range is 0..{len(lengths)-1}"
            )

        points = (
            self._sparse_progress_points
        )

        x0, y0 = points[leg]
        x1, y1 = points[leg + 1]

        length = float(
            lengths[leg]
        )

        qx = float(x)
        qy = float(y)

        vx = (
            x1 - x0
        )

        vy = (
            y1 - y0
        )

        t = (
            (
                (qx - x0) * vx
                + (qy - y0) * vy
            )
            / (length * length)
        )

        t = max(
            0.0,
            min(
                1.0,
                float(t),
            ),
        )

        px = (
            x0
            + t * vx
        )

        py = (
            y0
            + t * vy
        )

        lateral = math.hypot(
            qx - px,
            qy - py,
        )

        remaining = (
            (1.0 - t) * length
            + self._sparse_progress_tails[leg]
            + lateral
        )

        return (
            max(
                0.0,
                float(remaining),
            ),
            float(lateral),
            float(t),
        )


    def geodesic_distance(
        self,
        x,
        y,
        max_snap_m=0.20,
    ):
        """
        Backward-compatible B2 progress interface.

        Despite the historical method name, runtime progress is now
        the continuous safety-reference-path remaining distance.

        The underlying reference path still comes from the validated
        0.45-m safety-feasible geodesic topology.

        The second return value is the continuous lateral projection
        error rather than a discrete grid-cell snap distance.
        """

        # Retained only for API compatibility with existing callers.
        _ = max_snap_m

        distance, lateral, _ = (
            self.reference_path_distance(
                x,
                y,
            )
        )

        return (
            float(distance),
            float(lateral),
        )


    def progress_delta(
        self,
        previous_xy,
        current_xy,
    ):
        d_prev, snap_prev = (
            self.geodesic_distance(
                previous_xy[0],
                previous_xy[1],
            )
        )

        d_curr, snap_curr = (
            self.geodesic_distance(
                current_xy[0],
                current_xy[1],
            )
        )

        return {
            "previous_distance": d_prev,
            "current_distance": d_curr,
            "delta": d_prev - d_curr,
            "previous_snap": snap_prev,
            "current_snap": snap_curr,
        }

    # --------------------------------------------------------------
    # Safety-feasible reference path
    # --------------------------------------------------------------

    def _trace_reference_path(self):
        current = self.start_cell
        goal = self.goal_cell

        path = [current]

        limit = (
            self.safe_free.size
            + 1
        )

        for _ in range(limit):

            if current == goal:
                return path

            j, i = current
            current_d = float(
                self.safe_distance[j, i]
            )

            candidates = []

            for nj, ni, _ in (
                self._valid_neighbors(
                    self.safe_free,
                    j,
                    i,
                )
            ):
                nd = float(
                    self.safe_distance[nj, ni]
                )

                if (
                    math.isfinite(nd)
                    and nd < current_d - 1e-9
                ):
                    candidates.append(
                        (nd, nj, ni)
                    )

            if not candidates:
                raise RuntimeError(
                    f"{self.name}: could not trace "
                    "safety reference path"
                )

            _, nj, ni = min(candidates)

            current = (nj, ni)
            path.append(current)

        raise RuntimeError(
            f"{self.name}: reference-path loop"
        )



    # --------------------------------------------------------------
    # Sparse safety-feasible navigation route
    # --------------------------------------------------------------

    @staticmethod
    def _supercover_line(j0, i0, j1, i1):
        """
        Grid cells touched by a straight segment.
        Conservative supercover form for LOS checking.
        """
        cells = []

        dj = j1 - j0
        di = i1 - i0

        nj = abs(dj)
        ni = abs(di)

        sj = 1 if dj > 0 else -1 if dj < 0 else 0
        si = 1 if di > 0 else -1 if di < 0 else 0

        j = j0
        i = i0

        cells.append((j, i))

        cj = 0
        ci = 0

        while cj < nj or ci < ni:

            left = (
                (1 + 2 * ci) * nj
            )

            right = (
                (1 + 2 * cj) * ni
            )

            if left == right:
                i += si
                j += sj
                ci += 1
                cj += 1

            elif left < right:
                i += si
                ci += 1

            else:
                j += sj
                cj += 1

            cells.append((j, i))

        return cells

    def _safe_line_of_sight(
        self,
        a,
        b,
    ):
        for j, i in self._supercover_line(
            a[0],
            a[1],
            b[0],
            b[1],
        ):
            if not (
                self._inside(j, i)
                and self.safe_free[j, i]
            ):
                return False

        return True

    def navigation_cells(self):
        """
        Greedily compress the 0.45 m safety reference path.

        Consecutive returned cells have direct line-of-sight entirely
        inside safe_free, preventing long footprint-only shortcuts.
        """
        path = self.reference_cells

        if len(path) <= 2:
            return list(path)

        out = [
            path[0]
        ]

        anchor = 0

        while anchor < len(path) - 1:

            best = anchor + 1

            # Select the farthest reference cell that remains
            # directly visible through the 0.45 m safety-free set.
            for k in range(
                anchor + 1,
                len(path),
            ):
                if self._safe_line_of_sight(
                    path[anchor],
                    path[k],
                ):
                    best = k
                else:
                    break

            if best <= anchor:
                raise RuntimeError(
                    f"{self.name}: navigation route "
                    "compression stalled"
                )

            out.append(
                path[best]
            )

            anchor = best

        return out

    def navigation_waypoints(self):
        """
        Return:
            [(name, x, y, yaw), ...]

        Final element is always named G.
        """
        cells = self.navigation_cells()

        points = [
            self.cell_to_world(j, i)
            for j, i in cells
        ]

        waypoints = []

        # Do not return START itself as a Nav2 goal.
        for k in range(
            1,
            len(points),
        ):
            x, y = points[k]

            if k < len(points) - 1:
                nx, ny = points[k + 1]

                yaw = math.atan2(
                    ny - y,
                    nx - x,
                )

                name = f"P{k}"

            else:
                yaw = float(
                    self.goal[2]
                )

                name = "G"

                # Preserve the exact metadata goal.
                x = float(
                    self.goal[0]
                )
                y = float(
                    self.goal[1]
                )

            waypoints.append(
                (
                    name,
                    float(x),
                    float(y),
                    float(yaw),
                )
            )

        if not waypoints:
            raise RuntimeError(
                f"{self.name}: empty navigation waypoint set"
            )

        return waypoints


def list_environment_names(root):
    root = Path(root).expanduser().resolve()

    names = []

    for p in sorted(root.glob("*.json")):

        if p.name == "manifest.json":
            continue

        meta = json.loads(
            p.read_text()
        )

        name = str(
            meta.get("name", "")
        )

        if (
            name.startswith("train_")
            or name == "eval_building"
        ):
            names.append(name)

    return names
