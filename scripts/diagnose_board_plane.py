#!/usr/bin/env python3
"""Diagnose whether FAST-Calib's LiDAR board plane contains four clean holes."""

from __future__ import annotations

import argparse
import math
import os
from collections import deque
from pathlib import Path

import cv2
import numpy as np


GEOMETRY_TOLERANCE_M = 0.08


def read_ascii_ply_xyz(path: Path) -> np.ndarray:
    vertex_count = None
    header_lines = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            header_lines += 1
            line = line.strip()
            if line.startswith("element vertex "):
                vertex_count = int(line.split()[-1])
            if line == "end_header":
                break

    if vertex_count is None:
        raise ValueError(f"{path} does not declare an element vertex count")

    data = np.loadtxt(path, skiprows=header_lines, max_rows=vertex_count, dtype=np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 3:
        raise ValueError(f"{path} does not contain xyz columns")
    return data[:, :3]


def parse_simple_ros_params(path: Path) -> dict[str, float]:
    params: dict[str, float] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, value = [part.strip() for part in line.split(":", 1)]
        try:
            params[key] = float(value)
        except ValueError:
            continue
    return params


def connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=np.uint8)
    components: list[list[tuple[int, int]]] = []
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or seen[y, x]:
                continue
            queue: deque[tuple[int, int]] = deque([(x, y)])
            seen[y, x] = 1
            component: list[tuple[int, int]] = []
            while queue:
                cx, cy = queue.popleft()
                component.append((cx, cy))
                for nx in (cx - 1, cx, cx + 1):
                    for ny in (cy - 1, cy, cy + 1):
                        if nx == cx and ny == cy:
                            continue
                        if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = 1
                            queue.append((nx, ny))
            components.append(component)
    return components


def best_rectangle_group(
    candidates: list[tuple[float, float, float, float]],
    width: float,
    height: float,
) -> tuple[list[int], float] | tuple[None, None]:
    expected = sorted(
        [
            height,
            height,
            width,
            width,
            math.hypot(width, height),
            math.hypot(width, height),
        ]
    )
    best_indices = None
    best_error = None
    n = len(candidates)
    for a in range(n - 3):
        for b in range(a + 1, n - 2):
            for c in range(b + 1, n - 1):
                for d in range(c + 1, n):
                    indices = [a, b, c, d]
                    distances = []
                    for i, idx_a in enumerate(indices[:-1]):
                        for idx_b in indices[i + 1 :]:
                            ax, ay = candidates[idx_a][0], candidates[idx_a][1]
                            bx, by = candidates[idx_b][0], candidates[idx_b][1]
                            distances.append(math.hypot(ax - bx, ay - by))
                    distances.sort()
                    errors = [abs(got - want) for got, want in zip(distances, expected)]
                    mean_error = sum(errors) / len(errors)
                    max_error = max(errors)
                    if max_error > GEOMETRY_TOLERANCE_M:
                        continue
                    if best_error is None or mean_error < best_error:
                        best_error = mean_error
                        best_indices = indices
    return best_indices, best_error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", required=True, type=Path, help="FAST-Calib aligned_cloud.ply")
    parser.add_argument("--config", required=True, type=Path, help="FAST-Calib ROS parameter YAML")
    parser.add_argument("--out", required=True, type=Path, help="Output diagnostics directory")
    parser.add_argument("--resolution", type=float, default=0.01, help="Board-plane grid resolution in meters")
    args = parser.parse_args()

    points = read_ascii_ply_xyz(args.cloud)
    params = parse_simple_ros_params(args.config)
    width = params.get("delta_width_circles", 0.5)
    height = params.get("delta_height_circles", 0.4)
    radius = params.get("circle_radius", 0.12)
    resolution = args.resolution

    args.out.mkdir(parents=True, exist_ok=True)

    x = points[:, 0]
    y = points[:, 1]
    margin = max(radius * 1.8, 0.25)
    min_x, max_x = float(x.min() - margin), float(x.max() + margin)
    min_y, max_y = float(y.min() - margin), float(y.max() + margin)
    cols = max(1, int(math.ceil((max_x - min_x) / resolution)))
    rows = max(1, int(math.ceil((max_y - min_y) / resolution)))

    grid = np.zeros((rows, cols), dtype=np.uint16)
    ix = np.clip(((x - min_x) / resolution).astype(np.int32), 0, cols - 1)
    iy = np.clip(((y - min_y) / resolution).astype(np.int32), 0, rows - 1)
    np.add.at(grid, (iy, ix), 1)

    occupied = grid > 0
    kernel = np.ones((3, 3), dtype=np.uint8)
    board_mask = cv2.morphologyEx(occupied.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=2).astype(bool)
    empty_inside = board_mask & ~occupied
    components = connected_components(empty_inside)

    candidates = []
    for component in components:
        area = len(component) * resolution * resolution
        if area < math.pi * (radius * 0.25) ** 2 or area > math.pi * (radius * 1.8) ** 2:
            continue
        xs = np.array([p[0] for p in component], dtype=np.float32)
        ys = np.array([p[1] for p in component], dtype=np.float32)
        cx = min_x + (float(xs.mean()) + 0.5) * resolution
        cy = min_y + (float(ys.mean()) + 0.5) * resolution
        equiv_radius = math.sqrt(area / math.pi)
        candidates.append((cx, cy, equiv_radius, area))

    radius_filtered_candidates = [
        candidate
        for candidate in candidates
        if radius * 0.45 <= candidate[2] <= radius * 1.55
    ]
    best_group, best_group_error = best_rectangle_group(radius_filtered_candidates, width, height)

    expected = np.array(
        [
            [-width / 2.0, height / 2.0],
            [width / 2.0, height / 2.0],
            [width / 2.0, -height / 2.0],
            [-width / 2.0, -height / 2.0],
        ],
        dtype=np.float32,
    )
    centroid = np.array([float(x.mean()), float(y.mean())], dtype=np.float32)
    expected += centroid

    image = np.full((rows, cols, 3), 255, dtype=np.uint8)
    image[board_mask] = (230, 230, 230)
    image[occupied] = (30, 30, 30)
    image[empty_inside] = (210, 240, 255)

    def to_px(px: float, py: float) -> tuple[int, int]:
        gx = int(round((px - min_x) / resolution))
        gy = int(round((py - min_y) / resolution))
        return gx, gy

    for cx, cy, equiv_radius, _ in candidates:
        gx, gy = to_px(cx, cy)
        rr = max(2, int(round(equiv_radius / resolution)))
        cv2.circle(image, (gx, gy), rr, (0, 140, 255), 2)
        cv2.circle(image, (gx, gy), 2, (0, 0, 255), -1)

    if best_group is not None:
        group_points = [radius_filtered_candidates[i] for i in best_group]
        for i, point_a in enumerate(group_points[:-1]):
            for point_b in group_points[i + 1 :]:
                ax, ay = to_px(point_a[0], point_a[1])
                bx, by = to_px(point_b[0], point_b[1])
                cv2.line(image, (ax, ay), (bx, by), (0, 180, 0), 1)

    for ex, ey in expected:
        gx, gy = to_px(float(ex), float(ey))
        cv2.drawMarker(image, (gx, gy), (255, 0, 0), cv2.MARKER_CROSS, 18, 2)

    image = cv2.flip(image, 0)
    cv2.imwrite(str(args.out / "board_plane_hole_diagnostic.png"), image)

    report_path = args.out / "board_plane_hole_diagnostic.txt"
    with report_path.open("w", encoding="utf-8") as f:
        f.write(f"cloud: {args.cloud}\n")
        f.write(f"points: {len(points)}\n")
        f.write(f"resolution_m: {resolution:.4f}\n")
        f.write(f"expected_width_m: {width:.4f}\n")
        f.write(f"expected_height_m: {height:.4f}\n")
        f.write(f"expected_radius_m: {radius:.4f}\n")
        f.write(f"candidate_empty_regions: {len(candidates)}\n")
        for i, (cx, cy, equiv_radius, area) in enumerate(candidates, 1):
            f.write(f"candidate_{i}: x={cx:.4f} y={cy:.4f} equiv_radius={equiv_radius:.4f} area={area:.5f}\n")
        f.write(f"radius_filtered_candidates: {len(radius_filtered_candidates)}\n")
        if best_group is not None:
            f.write(f"best_rectangle_group_mean_error_m: {best_group_error:.4f}\n")
            for rank, candidate_idx in enumerate(best_group, 1):
                cx, cy, equiv_radius, area = radius_filtered_candidates[candidate_idx]
                f.write(
                    f"group_{rank}: x={cx:.4f} y={cy:.4f} "
                    f"equiv_radius={equiv_radius:.4f} area={area:.5f}\n"
                )
        verdict = "pass" if best_group is not None else "fail"
        f.write(f"verdict: {verdict}\n")
        if verdict == "fail":
            f.write("reason: no four radius-compatible empty regions matched the target rectangle geometry\n")

    print(report_path)
    print(args.out / "board_plane_hole_diagnostic.png")
    print(f"candidate_empty_regions={len(candidates)}")
    print(f"radius_filtered_candidates={len(radius_filtered_candidates)}")
    print(f"verdict={'pass' if best_group is not None else 'fail'}")
    return 0 if best_group is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
