# Corridor Incremental Pipeline

This repository contains scripts for an incremental corridor processing pipeline that consumes pointcloud batch PLY files, runs RANSAC-based wall segmentation, generates bounding boxes, merges results, and exports IFCs.

**Default layout**
- Pointcloud batches: `<ROOT>/4th_Floor_whole_corridor/pointcloud_batch_*.ply`
- Result scripts: `<ROOT>/result1_real/*.py` (e.g. `vertical_ransac.py`)
- Outputs: `<ROOT>/output_ani/`

By default the code expects the root directory to be: `~/ptv3/sonata_pipeline`.

**Make the root configurable**
`run_corridor_incremental.py` accepts a root path via either an environment variable or a CLI argument:

- Environment variable: `POINTCLOUD_ROOT`
- CLI argument: `--root`

The script checks `--root` first (if provided), otherwise it falls back to `POINTCLOUD_ROOT`, and finally to the default `~/ptv3/sonata_pipeline`.

Examples:

- Using environment variable (zsh):

```bash
export POINTCLOUD_ROOT="$HOME/ptv3/sonata_pipeline"
python3 run_corridor_incremental.py
```

- Using CLI argument:

```bash
python3 run_corridor_incremental.py --root /path/to/your/project_root
```

**Dependencies**
- Python 3.8+
- numpy
- open3d

You can install required Python packages with pip:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install numpy open3d
```

**What `run_corridor_incremental.py` does**
- Iterates over `pointcloud_batch_*.ply` files in the batches folder
- Runs `vertical_ransac.py` on each batch to detect wall segments
- Runs `make_wall_bboxes_from_segments.py` to generate bounding boxes per batch
- Merges bounding boxes using `merge_all_bboxes.py`
- Exports IFC with `csv_to_ifc_from_bboxes.py`
- Produces cumulative pointclouds and a folder with RANSAC segment PLY files

**Troubleshooting**
- If you see `No batches found in ...` ensure the `--root`/`POINTCLOUD_ROOT` points to a folder containing `4th_Floor_whole_corridor` with `pointcloud_batch_*.ply` files.
- If `open3d` import fails, install it with `pip install open3d` or follow your platform-specific installation instructions.

**Notes**
- The code will call the helper scripts in `result1_real/` relative to the chosen root. Keep those scripts in place or update the `RESULT_DIR` in `run_corridor_incremental.py`.

If you want, I can also:
- Add a small check at runtime to validate the presence of the helper scripts and give clearer error messages.
- Update other scripts to accept the same `--root` pattern for consistency.

Happy to make further changes or run a quick dry-run if you provide a small sample of your pointclouds.