#!/usr/bin/env python3
"""
segment_walls_from_batch_t2.py

Wrapper that combines:
  1) keep_vertical_auto_strict.py  → filter to strict vertical walls
  2) RANSAC.py                     → segment walls into wall_XXX.ply

Usage example:

    conda activate ptv3
    cd ~/ptv3/sonata_pipeline/result1_real

    python3 segment_walls_from_batch_t2.py \
        ../real_pc_data/Walls_only.ply \
        ../output/walls_from_Walls_only

This will:
  - create ../output/Walls_only_vertical_strict.ply
  - fill ../output/walls_from_Walls_only/ with wall_XXX.ply + walls_all_colored.ply
"""

import argparse
import subprocess
from pathlib import Path


# ---------------------------------------------------------------------
# Paths to helper scripts (same folder as this file)
# ---------------------------------------------------------------------
THIS_DIR = Path(__file__).resolve().parent

KEEP_VERTICAL_SCRIPT = THIS_DIR / "keep_vertical_auto_strict.py"
RANSAC_SCRIPT = THIS_DIR / "RANSAC.py"


def run(cmd):
    """Print and execute a subprocess command."""
    print("[CMD]", " ".join(str(c) for c in cmd))
    subprocess.check_call([str(c) for c in cmd])


def get_python_cmd():
    """Get the current Python executable to ensure scripts run in the same environment."""
    import sys
    return sys.executable


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Segment corridor walls from a batch point cloud by first "
            "keeping only strict vertical points, then running RANSAC.py."
        )
    )
    parser.add_argument(
        "input_ply",
        type=Path,
        help="Input point cloud (.ply), e.g. Walls_only.ply or batch_0001.ply",
    )
    parser.add_argument(
        "out_dir",
        type=Path,
        help="Output directory for wall_XXX.ply segments",
    )

    args = parser.parse_args()

    # --- sanity checks ---
    if not args.input_ply.is_file():
        raise FileNotFoundError(f"Input PLY not found: {args.input_ply}")

    if not KEEP_VERTICAL_SCRIPT.is_file():
        raise FileNotFoundError(
            f"keep_vertical_auto_strict.py not found at: {KEEP_VERTICAL_SCRIPT}"
        )

    if not RANSAC_SCRIPT.is_file():
        raise FileNotFoundError(
            f"RANSAC.py not found at: {RANSAC_SCRIPT}"
        )

    # Make sure output directory exists
    args.out_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------------------------------------------
    # 1) Run keep_vertical_auto_strict on the raw cloud
    #    -> produce a strict-vertical-only PLY inside the *same* out_dir
    # -----------------------------------------------------------------
    vertical_ply = args.out_dir / f"{args.input_ply.stem}_vertical_strict.ply"

    if vertical_ply.is_file():
        print(f"[INFO] Vertical-only PLY already exists, reusing: {vertical_ply}")
    else:
        print(f"[INFO] Creating vertical-only walls: {vertical_ply}")
        keep_cmd = [
            get_python_cmd(),
            str(KEEP_VERTICAL_SCRIPT),
            str(args.input_ply),
            "--out",
            str(vertical_ply),

            # You can tweak these if needed:
            "--up-axis", "z",
            "--voxel", "0.02",        # 0.0 = no downsample, 0.02 for speed
            "--plane-deg", "8.0",     # horizontals within 8° of up-axis
            "--vertical-deg", "6.0",  # verticals ≈ 90° ± 6°
            "--nb-neighbors", "50",
            "--std-ratio", "1.0",
            "--height-margin", "0.0",
        ]
        run(keep_cmd)

    # -----------------------------------------------------------------
    # 2) Run RANSAC.py wall segmentation on the vertical-only PLY
    #    -> fills out_dir with wall_XXX.ply and walls_all_colored.ply
    # -----------------------------------------------------------------
    print(f"[INFO] Running RANSAC wall segmentation on: {vertical_ply}")
    ransac_cmd = [
        get_python_cmd(),
        str(RANSAC_SCRIPT),
        str(vertical_ply),
        str(args.out_dir),
    ]
    run(ransac_cmd)

    print("[OK] Finished combined vertical-filter + RANSAC pipeline.")
    print(f"[OK] Segmented walls are in: {args.out_dir}")


if __name__ == "__main__":
    main()
