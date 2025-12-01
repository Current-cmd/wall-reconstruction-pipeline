#!/usr/bin/env python3
"""
Incremental pipeline for corridor using vertical_ransac.py:

For each batch i:
    1) vertical_ransac on batch_i  -> walls (per-batch)
    2) make bboxes for batch_i     -> wall_bboxes.csv (per-batch)
    3) merge all bboxes 1..i       -> global/bboxes_all_up_to_i.csv
    4) export IFC from merged CSV  -> global/walls_up_to_i.ifc
    5) merge vertical_strict 1..i  -> global/vertical_batches_1_to_i.ply
    6) copy all wall_*.ply from batches 1..i into
       global/walls_ransac_segments_1_to_i/
    7) print time for each batch in min / sec
"""

import subprocess
from pathlib import Path
import shutil
import time  # <-- for timing

import os
import argparse
import numpy as np
import open3d as o3d

# ---------------------------------------------------------------------
# Default paths (can be overridden via env var or CLI args)
# ---------------------------------------------------------------------
# The scripts expect pointclouds and related scripts to live under a
# root folder. By default this mirrors the previous behavior (the user's
# home `ptv3/sonata_pipeline`). You can override using the
# `POINTCLOUD_ROOT` environment variable or `--root` CLI argument.
DEFAULT_ROOT = Path.home() / "ptv3" / "sonata_pipeline"


def build_paths(root: Path):
    """Return a dict of derived paths for a given root."""
    root = Path(root)
    paths = {}
    paths["ROOT"] = root
    paths["BATCH_DIR"] = root / "4th_Floor_whole_corridor"
    paths["OUT_ROOT"] = root / "output_ani"
    paths["RESULT_DIR"] = root / "result1_real"
    # Derived script locations
    paths["T2_SCRIPT"] = paths["RESULT_DIR"] / "vertical_ransac.py"
    paths["BBOX_SCRIPT"] = paths["RESULT_DIR"] / "make_wall_bboxes_from_segments.py"
    paths["MERGE_SCRIPT"] = paths["RESULT_DIR"] / "merge_all_bboxes.py"
    paths["CSV2IFC_SCRIPT"] = paths["RESULT_DIR"] / "csv_to_ifc_from_bboxes.py"
    return paths


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def run(cmd):
    cmd = [str(c) for c in cmd]
    print("[CMD]", " ".join(cmd))
    subprocess.check_call(cmd)


def merge_pointclouds(ply_paths, out_path):
    """
    Merge multiple PLY point clouds into a single PLY.
    Keeps colors if present in the inputs.
    """
    all_pts = []
    all_cols = []
    has_colors = False

    ply_paths = [Path(p) for p in ply_paths if Path(p).is_file()]
    if not ply_paths:
        print(f"[WARN] No PLYs found to merge for {out_path}")
        return

    print(f"[INFO] Merging {len(ply_paths)} point clouds into {out_path}")

    for p in ply_paths:
        pcd = o3d.io.read_point_cloud(str(p))
        if len(pcd.points) == 0:
            continue

        pts = np.asarray(pcd.points)
        all_pts.append(pts)

        if pcd.has_colors():
            cols = np.asarray(pcd.colors)
            all_cols.append(cols)
            has_colors = True
        else:
            all_cols.append(None)

    if not all_pts:
        print(f"[WARN] All PLYs empty for {out_path}")
        return

    pts_all = np.vstack(all_pts)

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(pts_all)

    if has_colors:
        colors_list = []
        for pts, cols in zip(all_pts, all_cols):
            if cols is None:
                colors_list.append(np.ones((pts.shape[0], 3)))
            else:
                colors_list.append(cols)
        cols_all = np.vstack(colors_list)
        pcd_out.colors = o3d.utility.Vector3dVector(cols_all)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out_path), pcd_out)
    print(f"[OK] Wrote merged point cloud: {out_path}")


def format_time(seconds: float) -> str:
    """Return a human-readable time like '30 sec' or '1 min 20 sec'."""
    total_sec = int(round(seconds))
    minutes = total_sec // 60
    secs = total_sec % 60
    if minutes == 0:
        return f"{secs} sec"
    elif secs == 0:
        return f"{minutes} min"
    else:
        return f"{minutes} min {secs} sec"


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main(paths):
    BATCH_DIR = paths["BATCH_DIR"]
    OUT_ROOT = paths["OUT_ROOT"]
    T2_SCRIPT = paths["T2_SCRIPT"]
    BBOX_SCRIPT = paths["BBOX_SCRIPT"]
    MERGE_SCRIPT = paths["MERGE_SCRIPT"]
    CSV2IFC_SCRIPT = paths["CSV2IFC_SCRIPT"]

    batches = sorted(BATCH_DIR.glob("pointcloud_batch_*.ply"))
    if not batches:
        print(f"[ERROR] No batches found in {BATCH_DIR}")
        return

    OUT_ROOT.mkdir(exist_ok=True)
    global_dir = OUT_ROOT / "global"
    global_dir.mkdir(exist_ok=True)

    total_start = time.time()

    for idx, batch in enumerate(batches, start=1):
        batch_start = time.time()

        batch_name = batch.stem   # e.g. pointcloud_batch_0001_20251030_021932
        print("\n==============================")
        print(f"[INFO] Processing up to batch {idx}: {batch_name}")
        print("==============================")

        # ---------- 1) segment walls for this batch (vertical_ransac) ----------
        walls_dir = OUT_ROOT / f"{batch_name}_walls_t2"
        walls_dir.mkdir(exist_ok=True)
        if not any(walls_dir.glob("wall_*.ply")):
            run(["python3", T2_SCRIPT, batch, walls_dir])
        else:
            print(f"[INFO] Walls already exist for {batch_name}, skipping vertical_ransac.")

        # ---------- 2) make bounding boxes for this batch ----------
        bbox_dir = OUT_ROOT / f"{batch_name}_bboxes"
        bbox_dir.mkdir(exist_ok=True)
        bbox_csv = bbox_dir / "wall_bboxes.csv"

        if not bbox_csv.exists():
            run(["python3", BBOX_SCRIPT, walls_dir, bbox_dir])
        else:
            print(f"[INFO] BBoxes already exist for {batch_name}, skipping.")

        # ---------- 3) merge all bboxes seen so far ----------
        pattern = str(OUT_ROOT / "*_bboxes" / "wall_bboxes.csv")
        merged_csv = global_dir / f"bboxes_all_up_to_{idx:02d}.csv"
        run(["python3", MERGE_SCRIPT, pattern, merged_csv])

        # ---------- 4) export IFC from merged CSV ----------
        ifc_path = global_dir / f"walls_up_to_{idx:02d}.ifc"
        run(["python3", CSV2IFC_SCRIPT, merged_csv, ifc_path])
        print(f"[INFO] Finished IFC up to batch {idx}: {ifc_path}")

        # ---------- 5) cumulative vertical_strict point cloud ----------
        vertical_plys = []
        for b in batches[:idx]:
            bname = b.stem
            b_walls_dir = OUT_ROOT / f"{bname}_walls_t2"
            v_ply = b_walls_dir / f"{bname}_vertical_strict.ply"
            if v_ply.is_file():
                vertical_plys.append(v_ply)

        cum_vertical_out = global_dir / f"vertical_batches_1_to_{idx:02d}.ply"
        if vertical_plys:
            merge_pointclouds(vertical_plys, cum_vertical_out)
        else:
            print(f"[WARN] No vertical_strict PLYs found for cumulative up to {idx}")

        # ---------- 6) cumulative RANSAC segments (per-wall PLYs) ----------
        cum_seg_dir = global_dir / f"walls_ransac_segments_1_to_{idx:02d}"

        # Clear existing dir contents to avoid duplicates, then recreate
        if cum_seg_dir.exists():
            for old in cum_seg_dir.glob("*.ply"):
                old.unlink()
        else:
            cum_seg_dir.mkdir(parents=True, exist_ok=True)

        # Copy all wall_*.ply from batches 1..idx into this folder
        for b in batches[:idx]:
            bname = b.stem
            b_walls_dir = OUT_ROOT / f"{bname}_walls_t2"
            for wall_ply in sorted(b_walls_dir.glob("wall_*.ply")):
                # Name pattern: batchname__wall_001.ply etc.
                target_name = f"{bname}__{wall_ply.name}"
                target_path = cum_seg_dir / target_name
                shutil.copy2(wall_ply, target_path)

        print(f"[INFO] Cumulative RANSAC segments up to batch {idx}: {cum_seg_dir}")

        # ---------- 7) print time for this batch ----------
        batch_elapsed = time.time() - batch_start
        print(f"[TIME] Batch {idx} finished in {format_time(batch_elapsed)}")

    total_elapsed = time.time() - total_start
    print("\n======================================")
    print(f"[TIME] All batches finished in {format_time(total_elapsed)}")
    print("======================================\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run incremental corridor pipeline. Root can be set via env POINTCLOUD_ROOT or --root"
    )
    parser.add_argument("--root", help="Root folder containing pointcloud batches and result scripts",
                        default=os.environ.get("POINTCLOUD_ROOT", str(DEFAULT_ROOT)))
    args = parser.parse_args()

    root_path = Path(args.root)
    paths = build_paths(root_path)
    print(f"[INFO] Using root: {paths['ROOT']}")
    main(paths)
