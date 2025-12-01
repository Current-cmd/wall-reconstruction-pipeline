#!/usr/bin/env python3
"""
Create axis-aligned bounding boxes (wireframe only) for wall segments.

Usage:
    python make_wall_bboxes_from_segments.py <segments_dir> <out_dir>

Example:
    python make_wall_bboxes_from_segments.py \
        output/walls_segments_merged \
        output/bounding_boxes
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import open3d as o3d


def compute_aabb(points):
    """
    Compute an axis-aligned bounding box from a point array.
    Returns a dict with min/max, center and size.
    """
    pts = np.asarray(points)
    if pts.size == 0:
        return None

    min_xyz = pts.min(axis=0)
    max_xyz = pts.max(axis=0)
    center = 0.5 * (min_xyz + max_xyz)
    size = max_xyz - min_xyz

    return {
        "min_x": float(min_xyz[0]),
        "min_y": float(min_xyz[1]),
        "min_z": float(min_xyz[2]),
        "max_x": float(max_xyz[0]),
        "max_y": float(max_xyz[1]),
        "max_z": float(max_xyz[2]),
        "center_x": float(center[0]),
        "center_y": float(center[1]),
        "center_z": float(center[2]),
        "dx": float(size[0]),
        "dy": float(size[1]),
        "dz": float(size[2]),
    }


def write_boxes_to_obj(box_rows, out_obj_path):
    """
    Write all boxes into a single OBJ file as wireframe (edges only).
    This way they appear transparent in CloudCompare.
    """
    with open(out_obj_path, "w") as f:
        f.write("# Wireframe axis-aligned wall bounding boxes\n")
        v_offset = 0

        for row in box_rows:
            wid = int(row["wall_id"])
            xmin, ymin, zmin = row["min_x"], row["min_y"], row["min_z"]
            xmax, ymax, zmax = row["max_x"], row["max_y"], row["max_z"]

            # 8 vertices of the box
            verts = [
                (xmin, ymin, zmin),  # 1
                (xmax, ymin, zmin),  # 2
                (xmax, ymax, zmin),  # 3
                (xmin, ymax, zmin),  # 4
                (xmin, ymin, zmax),  # 5
                (xmax, ymin, zmax),  # 6
                (xmax, ymax, zmax),  # 7
                (xmin, ymax, zmax),  # 8
            ]

            f.write(f"\n# wall {wid:03d}\n")
            for v in verts:
                f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

            v1 = v_offset + 1
            v2 = v_offset + 2
            v3 = v_offset + 3
            v4 = v_offset + 4
            v5 = v_offset + 5
            v6 = v_offset + 6
            v7 = v_offset + 7
            v8 = v_offset + 8

            # Declare a group for this wall
            f.write(f"g wall_{wid:03d}\n")

            # 12 edges of the box as OBJ 'l' (line) elements
            edges = [
                (v1, v2), (v2, v3), (v3, v4), (v4, v1),  # bottom rectangle
                (v5, v6), (v6, v7), (v7, v8), (v8, v5),  # top rectangle
                (v1, v5), (v2, v6), (v3, v7), (v4, v8),  # vertical edges
            ]
            for a, b in edges:
                f.write(f"l {a} {b}\n")

            v_offset += 8


def main(segments_dir, out_dir):
    if not os.path.isdir(segments_dir):
        print(f"[ERROR] Segments dir not found: {segments_dir}")
        sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)

    # Only per-wall files, skip combined cloud
    ply_files = sorted(glob.glob(os.path.join(segments_dir, "wall_*.ply")))
    if not ply_files:
        print(f"[ERROR] No wall_XXX.ply files found in {segments_dir}")
        sys.exit(1)

    print(f"[INFO] Found {len(ply_files)} wall segments in {segments_dir}")

    rows = []
    for idx, ply_path in enumerate(ply_files, start=1):
        name = os.path.basename(ply_path)
        pcd = o3d.io.read_point_cloud(ply_path)
        pts = np.asarray(pcd.points)

        if pts.size == 0:
            print(f"[WARN] {name} has no points, skipping")
            continue

        box = compute_aabb(pts)
        if box is None:
            print(f"[WARN] Could not compute AABB for {name}, skipping")
            continue

        box.update(
            {
                "wall_id": idx,
                "ply_file": name,
                "n_points": int(len(pts)),
            }
        )
        rows.append(box)

    if not rows:
        print("[ERROR] No valid walls to make boxes from.")
        sys.exit(1)

    df = pd.DataFrame(rows).sort_values("wall_id")

    # CSV summary
    csv_path = os.path.join(out_dir, "wall_bboxes.csv")
    df.to_csv(csv_path, index=False)
    print(f"[OK] Wrote CSV with {len(df)} boxes: {csv_path}")

    # OBJ wireframe
    obj_path = os.path.join(out_dir, "wall_bboxes.obj")
    write_boxes_to_obj(df.to_dict("records"), obj_path)
    print(f"[OK] Wrote OBJ with {len(df)} wireframe boxes: {obj_path}")

    print("[DONE] Bounding boxes generated successfully.")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            "Usage:\n"
            "  python make_wall_bboxes_from_segments.py <segments_dir> <out_dir>\n\n"
            "Example:\n"
            "  python make_wall_bboxes_from_segments.py "
            "output/walls_segments_merged output/bounding_boxes"
        )
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
