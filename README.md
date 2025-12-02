# Wall Reconstruction Pipeline

Incremental pipeline for wall reconstruction from point clouds with **batch processing** and **real-time file monitoring**.

## Features

- 🔄 **Two Modes**: Batch (all files at once) or File Watcher (live monitoring)
- 📁 **Multi-Format**: PLY, LAS, LAZ with auto-conversion
- 🏗️ **RANSAC Segmentation**: Robust wall detection
- 📐 **IFC Export**: Industry-standard IFC4 format
- 📊 **Auto Visualization**: 2D views and statistics
- 🐳 **Docker Support**: Run in isolated containers

## Quick Start

### Option 1: Docker (Recommended)

```bash
# Build and run with one command
./docker-run.sh --build --batch-mode ./raw_pointCloud/Building_2_2nd_Floor

# Or use Docker Compose
docker-compose up
```

See [DOCKER.md](DOCKER.md) for detailed Docker instructions.

### Option 2: Local Installation

```bash
conda activate open3d_project_env
pip install open3d laspy matplotlib ifcopenshell
```

### Batch Mode (Process All Files)
```bash
python run_corridor_incremental.py --root . --batches /path/to/scans
```

### File Watcher Mode (Live Monitoring)
```bash
python run_corridor_incremental.py --root . --watch /path/to/scans --interval 5
# Press Ctrl+C to stop and finalize
```

## Usage

### Command-Line Arguments

| Argument | Description | Required | Default |
|----------|-------------|----------|---------|
| `--root` | Root folder with scripts | **Yes** | - |
| `--batches` | Directory with point clouds (batch mode) | Yes* | - |
| `--watch` | Directory to monitor (watcher mode) | Yes* | - |
| `--interval` | Poll interval in seconds (watcher only) | No | 5.0 |

*One of `--batches` or `--watch` is required

**Note**: `--root` should typically be set to `.` (current directory) if scripts are in the same folder.

### Mode Comparison

| Feature | Batch Mode | File Watcher Mode |
|---------|-----------|-------------------|
| **Processing** | All files at once | As files arrive |
| **Use Case** | Complete datasets | Live scanning |
| **Control** | Runs to completion | Stop with Ctrl+C |
| **Best For** | Post-processing | Real-time monitoring |

**Batch Mode** - Use when all files are ready:
```bash
python run_corridor_incremental.py --root . --batches /data/Building_Floor_2
```

**File Watcher Mode** - Use for live scanning:
```bash
python run_corridor_incremental.py --root . --watch /data/live_scans --interval 3
```

### Examples

```bash
# Process a building floor (scripts in current directory)
python run_corridor_incremental.py --root . --batches /data/Building_2_2nd_Floor

# Monitor live scanning (check every 3 seconds)
python run_corridor_incremental.py --root . --watch /data/live --interval 3

# Custom root directory (scripts elsewhere)
python run_corridor_incremental.py --root /custom/scripts --batches /data/scans
```

## Output Structure

```
output_[folder_name]/
├── batch_01_scan_001/
│   ├── walls/                        # Wall segments
│   ├── wall_bboxes.csv              # Bounding boxes
│   ├── walls_cumulative.ifc         # IFC up to this batch
│   └── vertical_walls_cumulative.ply
├── batch_02_scan_002/
│   └── ... (same structure)
├── walls_final.ifc                   # Final IFC with all walls
└── visualizations/
    ├── final_top_view.png
    ├── final_side_view.png
    ├── final_front_view.png
    └── point_cloud_stats.txt
```

## Processing Pipeline

Each batch goes through:
1. **Vertical Filtering** - Extract vertical wall points
2. **RANSAC Segmentation** - Detect wall planes
3. **Bounding Boxes** - Generate wall geometry
4. **Cumulative Merge** - Combine with previous batches
5. **IFC Generation** - Create/update IFC model
6. **Visualization** - Generate views (final batch only)

## File Format Support

- **PLY**: Point cloud with XYZ and optional RGB
- **LAS/LAZ**: ASPRS format (auto-converted to PLY)

Files are sorted numerically by first number in filename:
- `clip_001.ply` → 1
- `scan_042.ply` → 42

## Dependencies

**Required:**
- Python 3.8+
- open3d
- numpy
- ifcopenshell

**Optional:**
- laspy (for LAS/LAZ)
- matplotlib (for visualization)

**Docker:**
- Docker Engine 20.10+
- Docker Compose (optional)

## Configuration

**Note**: The `--root` argument is required and should point to the directory containing the pipeline scripts (`vertical_ransac.py`, `make_wall_bboxes_from_segments.py`, etc.). Use `--root .` if running from the script directory.

~~### Environment Variable~~
~~```bash~~
~~export POINTCLOUD_ROOT="/path/to/root"~~
~~python run_corridor_incremental.py --batches /data/scans~~
~~```~~

~~### Priority~~
~~`--root` argument > `POINTCLOUD_ROOT` env var > default path~~

## Troubleshooting

**No files found**
- Check directory path
- Verify file extensions (.ply, .las, .laz)

**LAS conversion fails**
```bash
pip install laspy
```

**Visualization missing**
```bash
pip install matplotlib
```

**File watcher not detecting**
- Increase `--interval` (e.g., `--interval 10`)
- Ensure files are fully written before detection

**Wrong processing order**
- Files sorted by first number in filename
- Use consistent naming: `scan_001.ply`, `scan_002.ply`
