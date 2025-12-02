#!/usr/bin/env python3
"""
Segment vertical walls from a batch point cloud using only:
1) Iterative plane RANSAC
2) DBSCAN per plane + merging nearby clusters
3) FINAL: aggressive merge of overlapping/duplicate wall labels

ASSUMPTION:
  - The input PLY is already pre-filtered as you like:
      * voxel + SOR (if desired),
      * floor/ceiling removed,
      * optionally vertical-only.
    This script DOES NOT do any of those three steps.

Usage (from ~/ptv3/sonata_pipeline):

  conda activate ptv3
  python3 result1_real/segment_walls_from_batch_t2.py \
      output5/batch1_to_5_verticals_strict.ply \
      output_sun
"""

import os
import argparse
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from itertools import combinations

# ------------------ PARAMETERS (tune here if needed) ------------------ #

VERTICAL_AXIS = np.array([0.0, 0.0, 1.0])  # z-up
MAX_WALL_TILT_DEG = 10.0    # how much a wall can tilt from vertical

RANSAC_DIST_THRESH = 0.03   # m; depends on point spacing
RANSAC_N = 3
RANSAC_NUM_ITERS = 2000
MIN_PLANE_INLIERS = 2000    # min points for accepting a plane
MAX_PLANES = 50

DBSCAN_EPS = 0.20           # m radius in plane coordinates (slightly larger)
DBSCAN_MIN_SAMPLES = 15
MIN_CLUSTER_POINTS = 200    # keep small walls too

MERGE_GAP_UV = 0.30         # m; max distance between clusters' 2D bboxes on plane

# FINAL merge across walls
ANGLE_MERGE_MAX_DEG = 60.0  # normals closer than this considered "similar"
MERGE_GAP_XY_FINAL = 0.30   # max gap between axis-aligned XY bboxes to merge
# --------------------------------------------------------------------- #


def angle_with_axis(normal: np.ndarray, axis: np.ndarray) -> float:
    """Return angle (deg) between vector and axis, symmetric for +/-."""
    normal = normal / np.linalg.norm(normal)
    axis = axis / np.linalg.norm(axis)
    cosang = np.clip(np.dot(normal, axis), -1.0, 1.0)
    return np.degrees(np.arccos(abs(cosang)))


def iterative_ransac_vertical_planes(pcd: o3d.geometry.PointCloud):
    """
    Iterative RANSAC: extract vertical wall planes from the given point cloud.

    IMPORTANT:
    - Assumes the cloud is already cleaned (no need for SOR/floor/vertical-only).
    - We only reject planes that are not wall-like (normal not ~horizontal) or
      too small (< MIN_PLANE_INLIERS).
    """
    print("[INFO] Starting iterative RANSAC for vertical planes...")
    planes = []
    remaining = pcd

    for i in range(MAX_PLANES):
        if len(remaining.points) < MIN_PLANE_INLIERS:
            print("[INFO] Not enough points left for more planes.")
            break

        plane_model, inliers = remaining.segment_plane(
            distance_threshold=RANSAC_DIST_THRESH,
            ransac_n=RANSAC_N,
            num_iterations=RANSAC_NUM_ITERS
        )
        a, b, c, d = plane_model
        normal = np.array([a, b, c], dtype=float)
        angle_deg = angle_with_axis(normal, VERTICAL_AXIS)

        # vertical walls → normal nearly horizontal (angle ~ 90°)
        if angle_deg >= (90 - MAX_WALL_TILT_DEG) and len(inliers) >= MIN_PLANE_INLIERS:
            plane_pcd = remaining.select_by_index(inliers)
            planes.append((plane_model, plane_pcd))
            remaining = remaining.select_by_index(inliers, invert=True)
            print(f"  Plane {i+1}: {len(plane_pcd.points)} pts "
                  f"(angle to vertical axis ≈ {angle_deg:.2f} deg). "
                  f"Remaining: {len(remaining.points)}")
        else:
            print(f"  Plane {i+1}: rejected as wall "
                  f"(angle ≈ {angle_deg:.2f} deg, inliers={len(inliers)})")
            break

    print(f"[INFO] Found {len(planes)} vertical planes before DBSCAN splitting.")
    return planes


def orthonormal_basis_from_normal(n: np.ndarray):
    """Given a normal n, return two orthonormal vectors u,v on the plane."""
    n = n / np.linalg.norm(n)
    if abs(n[0]) < 0.9:
        a = np.array([1.0, 0.0, 0.0])
    else:
        a = np.array([0.0, 1.0, 0.0])
    u = a - np.dot(a, n) * n
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    v /= np.linalg.norm(v)
    return u, v


def merge_clusters_by_bbox(uv: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """
    Merge DBSCAN clusters on the same plane whose 2D bounding boxes
    overlap or are within MERGE_GAP_UV in both u and v directions.
    """
    uniq = [lab for lab in np.unique(labels) if lab >= 0]
    if len(uniq) <= 1:
        return labels

    # bounding boxes for each cluster
    bboxes = {}
    for lab in uniq:
        pts = uv[labels == lab]
        min_u, min_v = pts.min(axis=0)
        max_u, max_v = pts.max(axis=0)
        bboxes[lab] = (min_u, max_u, min_v, max_v)

    # union–find structure
    parent = {lab: lab for lab in uniq}

    def find(x):
        while parent[x] != x:
            x = parent[x]
        return x

    def union(x, y):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[ry] = rx

    # merge clusters whose bboxes touch or are close
    for i, j in combinations(uniq, 2):
        min_u1, max_u1, min_v1, max_v1 = bboxes[i]
        min_u2, max_u2, min_v2, max_v2 = bboxes[j]

        # gap between intervals (0 if overlapping)
        gap_u = max(0.0, max(min_u2 - max_u1, min_u1 - max_u2))
        gap_v = max(0.0, max(min_v2 - max_v1, min_v1 - max_v2))

        if gap_u < MERGE_GAP_UV and gap_v < MERGE_GAP_UV:
            union(i, j)

    # assign new compact group IDs
    new_labels = -np.ones_like(labels)
    group_map = {}
    next_id = 0

    for lab in uniq:
        root = find(lab)
        if root not in group_map:
            group_map[root] = next_id
            next_id += 1
        new_labels[labels == lab] = group_map[root]

    return new_labels


def split_plane_with_dbscan(plane_model, plane_pcd: o3d.geometry.PointCloud):
    """
    For a given wall plane, project its points into 2D (u,v), run DBSCAN,
    then merge nearby clusters on that plane and return cluster point clouds.
    """
    points = np.asarray(plane_pcd.points)
    a, b, c, d = plane_model
    n = np.array([a, b, c], dtype=float)
    n /= np.linalg.norm(n)

    # build plane basis
    u, v = orthonormal_basis_from_normal(n)
    uv = np.stack([points @ u, points @ v], axis=1)

    # DBSCAN in 2D plane coordinates
    db = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(uv)
    labels = db.labels_

    # merge nearby clusters on same plane
    labels = merge_clusters_by_bbox(uv, labels)

    unique_labels = [lab for lab in np.unique(labels) if lab >= 0]
    clusters = []
    for lab in unique_labels:
        mask = labels == lab
        if np.count_nonzero(mask) < MIN_CLUSTER_POINTS:
            continue
        cluster_points = points[mask]
        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        clusters.append(cluster_pcd)

    return clusters


def plane_from_points(points: np.ndarray):
    """Fit plane n·x + d = 0 via PCA; return (normal, d)."""
    centroid = points.mean(axis=0)
    pts_centered = points - centroid
    cov = np.dot(pts_centered.T, pts_centered) / pts_centered.shape[0]
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, np.argmin(eigvals)]
    normal /= np.linalg.norm(normal)
    d = -np.dot(normal, centroid)
    return normal, d


def post_merge_overlapping_walls(wall_list):
    """
    FINAL aggressive clean-up: merge any walls that

      - have similar orientation (angle between normals < ANGLE_MERGE_MAX_DEG)
      - AND whose axis-aligned XY bounding boxes overlap or are within
        MERGE_GAP_XY_FINAL.

    This removes duplicate/overlapping labels on the same physical wall
    (including inner/outer faces).
    """
    if len(wall_list) <= 1:
        return wall_list

    normals = []
    bboxes_xy = []

    for wall in wall_list:
        pts = np.asarray(wall.points)
        n, d = plane_from_points(pts)
        normals.append(n)
        min_xyz = pts.min(axis=0)
        max_xyz = pts.max(axis=0)
        bboxes_xy.append((min_xyz[0], max_xyz[0],
                          min_xyz[1], max_xyz[1]))  # X, Y

    normals = np.array(normals)

    # union–find over walls
    parent = {i: i for i in range(len(wall_list))}

    def find(i):
        while parent[i] != i:
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    # decide merges (orientation + XY overlap)
    for i, j in combinations(range(len(wall_list)), 2):
        ni, nj = normals[i], normals[j]
        cosang = np.clip(np.dot(ni, nj), -1.0, 1.0)
        ang = np.degrees(np.arccos(abs(cosang)))

        # skip cross-walls (~90 deg apart)
        if ang > ANGLE_MERGE_MAX_DEG:
            continue

        # XY bbox check
        min_x1, max_x1, min_y1, max_y1 = bboxes_xy[i]
        min_x2, max_x2, min_y2, max_y2 = bboxes_xy[j]

        gap_x = max(0.0, max(min_x2 - max_x1, min_x1 - max_x2))
        gap_y = max(0.0, max(min_y2 - max_y1, min_y1 - max_y2))

        if gap_x < MERGE_GAP_XY_FINAL and gap_y < MERGE_GAP_XY_FINAL:
            union(i, j)

    # build merged walls
    groups = {}
    for idx in range(len(wall_list)):
        root = find(idx)
        groups.setdefault(root, []).append(idx)

    merged_walls = []
    for group_indices in groups.values():
        pts_list = [np.asarray(wall_list[idx].points) for idx in group_indices]
        pts_all = np.vstack(pts_list)
        merged = o3d.geometry.PointCloud()
        merged.points = o3d.utility.Vector3dVector(pts_all)
        merged_walls.append(merged)

    print(f"[INFO] Post-merge reduced walls from {len(wall_list)} to "
          f"{len(merged_walls)}")
    return merged_walls


def colorize_and_save_walls(wall_list, out_dir: str):
    os.makedirs(out_dir, exist_ok=True)

    all_points = []
    all_colors = []

    for i, wall in enumerate(wall_list, start=1):
        color = np.random.rand(3)
        pts = np.asarray(wall.points)
        cols = np.tile(color, (pts.shape[0], 1))

        wall_colored = o3d.geometry.PointCloud()
        wall_colored.points = o3d.utility.Vector3dVector(pts)
        wall_colored.colors = o3d.utility.Vector3dVector(cols)

        fname = os.path.join(out_dir, f"wall_{i:03d}.ply")
        o3d.io.write_point_cloud(fname, wall_colored)
        print(f"[SAVE] {fname} ({len(pts)} pts)")

        all_points.append(pts)
        all_colors.append(cols)

    if all_points:
        all_points = np.vstack(all_points)
        all_colors = np.vstack(all_colors)
        all_pcd = o3d.geometry.PointCloud()
        all_pcd.points = o3d.utility.Vector3dVector(all_points)
        all_pcd.colors = o3d.utility.Vector3dVector(all_colors)
        merged_path = os.path.join(out_dir, "walls_all_colored.ply")
        o3d.io.write_point_cloud(merged_path, all_pcd)
        print(f"[SAVE] {merged_path} ({len(all_points)} pts)")


def main():
    parser = argparse.ArgumentParser(
        description="Segment vertical walls from a batch point cloud "
                    "using only RANSAC + DBSCAN + final merge "
                    "(no voxel/SOR, no floor/ceiling, no vertical-normal filter)."
    )
    parser.add_argument("input_ply", help="Input batch PLY file "
                                          "(already cleaned as you like)")
    parser.add_argument("output_dir",
                        help="Output folder for wall PLYs (will be created)")
    args = parser.parse_args()

    print(f"[INFO] Loading {args.input_ply}")
    pcd = o3d.io.read_point_cloud(args.input_ply)
    if len(pcd.points) == 0:
        raise RuntimeError("Loaded point cloud is empty!")

    # DIRECTLY: iterative RANSAC on the whole cloud
    planes = iterative_ransac_vertical_planes(pcd)

    # Split each plane into clusters and merge (per-plane)
    wall_clusters = []
    for j, (plane_model, plane_pcd) in enumerate(planes, start=1):
        print(f"[INFO] Splitting plane {j} with DBSCAN + per-plane merge...")
        clusters = split_plane_with_dbscan(plane_model, plane_pcd)
        print(f"       → {len(clusters)} wall cluster(s) kept.")
        wall_clusters.extend(clusters)

    print(f"[INFO] Total wall clusters before final merge: {len(wall_clusters)}")

    # FINAL: merge overlapping/duplicate walls (same orientation + XY overlap)
    all_walls = post_merge_overlapping_walls(wall_clusters)

    # Colorize & save
    colorize_and_save_walls(all_walls, args.output_dir)
    print("[DONE]")


if __name__ == "__main__":
    main()
