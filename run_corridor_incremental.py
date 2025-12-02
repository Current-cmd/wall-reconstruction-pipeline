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

Supports both .ply and .las/.laz file formats (auto-converts LAS to PLY)

Modes:
    - Batch Mode: Process all files at once (default)
    - File Watcher Mode: Continuously monitor directory for new files
"""

import subprocess
from pathlib import Path
import shutil
import time
import logging
from typing import List, Set

import os
import argparse
import numpy as np
import open3d as o3d

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

try:
    import laspy
    LAS_SUPPORT = True
except ImportError:
    LAS_SUPPORT = False
    print("[WARN] laspy not installed. LAS/LAZ support disabled. Install with: pip install laspy")

# ---------------------------------------------------------------------
# Default paths (can be overridden via env var or CLI args)
# ---------------------------------------------------------------------


def build_paths(root: Path):
    """Return a dict of derived paths for a given root."""
    root = Path(root)
    paths = {}
    paths["ROOT"] = root
    paths["BATCH_DIR"] = root / "4th_Floor_whole_corridor"
    paths["OUT_ROOT"] = root / "output_ani"
    paths["RESULT_DIR"] = root / "result1_real"
    # Derived script locations - check both result1_real and root
    paths["T2_SCRIPT"] = root / "vertical_ransac.py"
    paths["BBOX_SCRIPT"] = root / "make_wall_bboxes_from_segments.py"
    paths["MERGE_SCRIPT"] = root / "merge_all_bboxes.py"
    paths["CSV2IFC_SCRIPT"] = root / "csv_to_ifc_from_bboxes.py"
    return paths


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def run(cmd):
    cmd = [str(c) for c in cmd]
    print("[CMD]", " ".join(cmd))
    subprocess.check_call(cmd)


def get_python_cmd():
    """Get the current Python executable to ensure scripts run in the same environment."""
    import sys
    return sys.executable


def convert_las_to_ply(las_path: Path, output_dir: Path) -> Path:
    """
    Convert LAS/LAZ file to PLY format.
    
    Args:
        las_path: Path to input LAS/LAZ file
        output_dir: Directory to save converted PLY file
        
    Returns:
        Path to converted PLY file
    """
    if not LAS_SUPPORT:
        raise RuntimeError("laspy not installed. Cannot convert LAS files.")
    
    print(f"[INFO] Converting LAS to PLY: {las_path.name}")
    
    # Read LAS file
    las = laspy.read(str(las_path))
    
    # Extract coordinates
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Extract colors if available
    if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
        colors = np.vstack((las.red, las.green, las.blue)).transpose()
        # Normalize to 0-1 range
        colors = colors / 65535.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save as PLY
    output_dir.mkdir(parents=True, exist_ok=True)
    ply_path = output_dir / f"{las_path.stem}.ply"
    o3d.io.write_point_cloud(str(ply_path), pcd)
    
    print(f"[OK] Converted to PLY: {ply_path} ({len(points)} points)")
    
    return ply_path


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


class WallReconstructionPipeline:
    """
    Pipeline for incremental wall reconstruction from point clouds.
    
    Supports two modes:
    - Batch mode: Process all files at once
    - File watcher mode: Continuously monitor directory for new files
    """
    
    def __init__(self, paths: dict):
        """
        Initialize the pipeline.
        
        Args:
            paths: Dictionary containing all necessary paths
        """
        self.paths = paths
        self.processed_files: Set[Path] = set()
        self.batch_counter = 0
        self.total_start_time = time.time()
        
    def process_batch(self, ply_file: Path) -> None:
        """
        Process a single point cloud batch.
        
        Args:
            ply_file: Path to PLY file to process
        """
        batch_start = time.time()
        self.batch_counter += 1
        
        batch_name = ply_file.stem
        logger.info(f"Processing batch {self.batch_counter}: {batch_name}")
        
        OUT_ROOT = self.paths["OUT_ROOT"]
        T2_SCRIPT = self.paths["T2_SCRIPT"]
        BBOX_SCRIPT = self.paths["BBOX_SCRIPT"]
        MERGE_SCRIPT = self.paths["MERGE_SCRIPT"]
        CSV2IFC_SCRIPT = self.paths["CSV2IFC_SCRIPT"]
        
        # Create batch-specific directory
        batch_dir = OUT_ROOT / f"batch_{self.batch_counter:02d}_{batch_name}"
        batch_dir.mkdir(parents=True, exist_ok=True)
        
        # ---------- 1) Segment walls for this batch (vertical_ransac) ----------
        walls_dir = batch_dir / "walls"
        walls_dir.mkdir(exist_ok=True)
        if not any(walls_dir.glob("wall_*.ply")):
            logger.info(f"Running wall segmentation...")
            run([get_python_cmd(), T2_SCRIPT, ply_file, walls_dir])
        else:
            logger.info(f"Walls already exist for {batch_name}, skipping vertical_ransac.")
        
        # ---------- 2) Make bounding boxes for this batch ----------
        bbox_csv = batch_dir / "wall_bboxes.csv"
        if not bbox_csv.exists():
            logger.info(f"Creating bounding boxes...")
            run([get_python_cmd(), BBOX_SCRIPT, walls_dir, batch_dir])
        else:
            logger.info(f"BBoxes already exist for {batch_name}, skipping.")
        
        # ---------- 3) Merge all bboxes seen so far ----------
        pattern = str(OUT_ROOT / "batch_*" / "wall_bboxes.csv")
        merged_csv = batch_dir / "bboxes_cumulative.csv"
        logger.info(f"Merging bounding boxes (batches 1-{self.batch_counter})...")
        run([get_python_cmd(), MERGE_SCRIPT, pattern, merged_csv])
        
        # ---------- 4) Export IFC from merged CSV (cumulative) ----------
        ifc_path = batch_dir / "walls_cumulative.ifc"
        logger.info(f"Generating cumulative IFC...")
        run([get_python_cmd(), CSV2IFC_SCRIPT, merged_csv, ifc_path])
        logger.info(f"Cumulative IFC up to batch {self.batch_counter}: {ifc_path}")
        
        # ---------- 5) Cumulative vertical_strict point cloud ----------
        vertical_plys = []
        for i in range(1, self.batch_counter + 1):
            batch_dirs = sorted(OUT_ROOT.glob(f"batch_{i:02d}_*"))
            if batch_dirs:
                b_walls_dir = batch_dirs[0] / "walls"
                for v_ply in b_walls_dir.glob("*_vertical_strict.ply"):
                    if v_ply.is_file():
                        vertical_plys.append(v_ply)
                        break
        
        cum_vertical_out = batch_dir / "vertical_walls_cumulative.ply"
        if vertical_plys:
            logger.info(f"Merging cumulative point cloud...")
            merge_pointclouds(vertical_plys, cum_vertical_out)
        else:
            logger.warning(f"No vertical_strict PLYs found for cumulative up to {self.batch_counter}")
        
        # ---------- 6) Cumulative RANSAC segments (per-wall PLYs) ----------
        cum_seg_dir = batch_dir / "wall_segments_cumulative"
        cum_seg_dir.mkdir(parents=True, exist_ok=True)
        
        # Clear existing dir contents to avoid duplicates
        for old in cum_seg_dir.glob("*.ply"):
            old.unlink()
        
        # Copy all wall_*.ply from batches 1..idx into this folder
        for i in range(1, self.batch_counter + 1):
            batch_dirs = sorted(OUT_ROOT.glob(f"batch_{i:02d}_*"))
            if batch_dirs:
                b_walls_dir = batch_dirs[0] / "walls"
                b_name = batch_dirs[0].name
                for wall_ply in sorted(b_walls_dir.glob("wall_*.ply")):
                    target_name = f"{b_name}__{wall_ply.name}"
                    target_path = cum_seg_dir / target_name
                    shutil.copy2(wall_ply, target_path)
        
        logger.info(f"Cumulative wall segments up to batch {self.batch_counter}: {cum_seg_dir}")
        
        # ---------- 7) Print time for this batch ----------
        batch_elapsed = time.time() - batch_start
        logger.info(f"Batch {self.batch_counter} finished in {format_time(batch_elapsed)}")
    
    def _finalize_reconstruction(self) -> None:
        """
        Finalize the reconstruction by creating final IFC and visualizations.
        """
        OUT_ROOT = self.paths["OUT_ROOT"]
        CSV2IFC_SCRIPT = self.paths["CSV2IFC_SCRIPT"]
        
        logger.info("=" * 60)
        logger.info("FINALIZING RECONSTRUCTION")
        logger.info("=" * 60)
        
        # Create final IFC in main output folder
        last_batch_dir = sorted(OUT_ROOT.glob(f"batch_{self.batch_counter:02d}_*"))
        if last_batch_dir:
            final_csv = last_batch_dir[0] / "bboxes_cumulative.csv"
            final_ifc = OUT_ROOT / "walls_final.ifc"
            
            if final_csv.exists():
                logger.info("Creating final cumulative IFC...")
                run([get_python_cmd(), CSV2IFC_SCRIPT, final_csv, final_ifc])
                logger.info(f"Final IFC with all {self.batch_counter} batches: {final_ifc}")
        
        # Create final visualization
        try:
            create_final_visualization(OUT_ROOT)
        except Exception as e:
            logger.warning(f"Visualization failed: {e}")
        
        total_elapsed = time.time() - self.total_start_time
        
        logger.info("=" * 60)
        logger.info(f"All batches finished in {format_time(total_elapsed)}")
        logger.info(f"Processed {self.batch_counter} batches successfully")
        logger.info(f"Output directory: {OUT_ROOT}")
        logger.info("=" * 60)
    
    def run_file_watcher(self, watch_directory: Path, poll_interval: float = 5.0) -> None:
        """
        Run in file watcher mode (continuous monitoring).
        
        Monitors directory every poll_interval seconds for new PLY/LAS files.
        Automatically converts LAS/LAZ files to PLY before processing.
        
        Args:
            watch_directory: Directory to monitor for new files
            poll_interval: Seconds between directory scans (default: 5.0)
        """
        logger.info("=" * 60)
        logger.info("STARTING FILE WATCHER MODE")
        logger.info("=" * 60)
        logger.info(f"Monitoring: {watch_directory}")
        logger.info(f"Poll interval: {poll_interval} seconds")
        logger.info("Press Ctrl+C to stop...")
        logger.info("")
        
        try:
            while True:
                # Scan for new PLY and LAS files
                ply_files = sorted(watch_directory.glob("*.ply"))
                las_files = list(watch_directory.glob("*.las")) + list(watch_directory.glob("*.laz"))
                
                # Convert any new LAS files
                if las_files:
                    if not LAS_SUPPORT:
                        logger.error("Found LAS/LAZ files but laspy is not installed.")
                        logger.error("Install with: pip install laspy")
                    else:
                        converted_dir = watch_directory / "converted_ply"
                        for las_file in las_files:
                            if las_file not in self.processed_files:
                                try:
                                    logger.info(f"NEW LAS FILE DETECTED: {las_file.name}")
                                    ply_file = convert_las_to_ply(las_file, converted_dir)
                                    ply_files.append(ply_file)
                                    self.processed_files.add(las_file)
                                except Exception as e:
                                    logger.error(f"Failed to convert {las_file.name}: {e}")
                
                # Sort PLY files by numeric value
                import re
                def extract_number(filepath):
                    numbers = re.findall(r'\d+', filepath.stem)
                    return int(numbers[0]) if numbers else 0
                
                ply_files = sorted(ply_files, key=extract_number)
                
                # Process new PLY files
                for ply_file in ply_files:
                    if ply_file not in self.processed_files:
                        logger.info(f"\n{'='*60}")
                        logger.info(f"NEW FILE DETECTED: {ply_file.name}")
                        logger.info(f"{'='*60}")
                        
                        self.process_batch(ply_file)
                        self.processed_files.add(ply_file)
                
                # Wait before next scan
                time.sleep(poll_interval)
                
        except KeyboardInterrupt:
            logger.info("\n\nFile watcher stopped by user")
            if self.batch_counter > 0:
                self._finalize_reconstruction()
            else:
                logger.info("No files were processed")
    
    def run_batch_mode(self, input_files: List[Path]) -> None:
        """
        Run in batch mode (process all files at once).
        
        Args:
            input_files: List of PLY files to process
        """
        logger.info("=" * 60)
        logger.info("STARTING BATCH MODE")
        logger.info("=" * 60)
        logger.info(f"Files to process: {len(input_files)}")
        logger.info("")
        
        for i, ply_file in enumerate(input_files, 1):
            logger.info(f"\n{'='*60}")
            logger.info(f"PROCESSING FILE {i}/{len(input_files)}: {ply_file.name}")
            logger.info(f"{'='*60}")
            
            self.process_batch(ply_file)
        
        self._finalize_reconstruction()


def create_final_visualization(out_root: Path):
    """
    Create visualization of the final point cloud with wall bounding boxes overlaid.
    
    Args:
        out_root: Output root directory containing batch results
    """
    logger.info("")
    logger.info("=" * 60)
    logger.info("Creating final visualization...")
    logger.info("=" * 60)
    
    try:
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        import pandas as pd
    except ImportError:
        logger.warning("matplotlib or pandas not installed. Skipping visualization.")
        return
    
    # Find the last batch directory
    batch_dirs = sorted(out_root.glob("batch_*"))
    if not batch_dirs:
        logger.warning("No batch directories found for visualization")
        return
    
    last_batch_dir = batch_dirs[-1]
    
    # Find the final cumulative point cloud and bounding boxes
    final_ply = last_batch_dir / "vertical_walls_cumulative.ply"
    final_csv = last_batch_dir / "bboxes_cumulative.csv"
    
    if not final_ply.exists():
        logger.warning(f"Final point cloud not found: {final_ply}")
        return
    
    logger.info(f"Loading final point cloud: {final_ply.name}")
    pcd = o3d.io.read_point_cloud(str(final_ply))
    points = np.asarray(pcd.points)
    
    logger.info(f"Total points: {len(points):,}")
    
    # Load bounding boxes if available
    bboxes = None
    if final_csv.exists():
        logger.info(f"Loading bounding boxes: {final_csv.name}")
        bboxes = pd.read_csv(final_csv)
        logger.info(f"Total walls: {len(bboxes)}")
    
    # Create visualization directory
    vis_dir = out_root / "visualizations"
    vis_dir.mkdir(exist_ok=True)
    
    # Create 2D top view with bounding boxes
    logger.info("Creating 2D top view...")
    fig, ax = plt.subplots(figsize=(20, 16), dpi=150)
    
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Color by height
    scatter = ax.scatter(x, y, c=z, s=0.5, alpha=0.5, cmap='viridis', label='Wall Points')
    
    # Overlay bounding boxes if available
    if bboxes is not None:
        for idx, row in bboxes.iterrows():
            width = row['max_x'] - row['min_x']
            height = row['max_y'] - row['min_y']
            rect = Rectangle((row['min_x'], row['min_y']), width, height,
                           linewidth=2, edgecolor='red', facecolor='none',
                           label='Wall BBox' if idx == 0 else '')
            ax.add_patch(rect)
            
            # Add wall ID
            center_x = row['center_x']
            center_y = row['center_y']
            ax.text(center_x, center_y, f"W{row['wall_id']}", 
                   ha='center', va='center', fontsize=8, color='white',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='red', alpha=0.7))
    
    plt.colorbar(scatter, ax=ax, label='Height (m)')
    ax.set_xlabel('X (meters)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (meters)', fontsize=14, fontweight='bold')
    title = f'Final Wall Reconstruction - Top View\n({len(points):,} points'
    if bboxes is not None:
        title += f', {len(bboxes)} walls)'
    else:
        title += ')'
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    top_view_path = vis_dir / "final_top_view.png"
    plt.savefig(top_view_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    
    logger.info(f"Saved top view: {top_view_path}")
    
    # Create side view (XZ plane) with bounding boxes
    logger.info("Creating side view...")
    fig, ax = plt.subplots(figsize=(20, 10), dpi=150)
    
    scatter = ax.scatter(x, z, c=y, s=0.5, alpha=0.5, cmap='plasma', label='Wall Points')
    
    # Overlay bounding boxes if available
    if bboxes is not None:
        for idx, row in bboxes.iterrows():
            width = row['max_x'] - row['min_x']
            height = row['max_z'] - row['min_z']
            rect = Rectangle((row['min_x'], row['min_z']), width, height,
                           linewidth=2, edgecolor='cyan', facecolor='none',
                           label='Wall BBox' if idx == 0 else '')
            ax.add_patch(rect)
            
            # Add wall ID
            center_x = row['center_x']
            center_z = row['center_z']
            ax.text(center_x, center_z, f"W{row['wall_id']}", 
                   ha='center', va='center', fontsize=8, color='white',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='cyan', alpha=0.7))
    
    plt.colorbar(scatter, ax=ax, label='Y Position (m)')
    ax.set_xlabel('X (meters)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Height (meters)', fontsize=14, fontweight='bold')
    title = f'Final Wall Reconstruction - Side View (X-Z)\n({len(points):,} points'
    if bboxes is not None:
        title += f', {len(bboxes)} walls)'
    else:
        title += ')'
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    side_view_path = vis_dir / "final_side_view.png"
    plt.savefig(side_view_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    
    logger.info(f"Saved side view: {side_view_path}")
    
    # Create front view (YZ plane) with bounding boxes
    logger.info("Creating front view...")
    fig, ax = plt.subplots(figsize=(20, 10), dpi=150)
    
    scatter = ax.scatter(y, z, c=x, s=0.5, alpha=0.5, cmap='coolwarm', label='Wall Points')
    
    # Overlay bounding boxes if available
    if bboxes is not None:
        for idx, row in bboxes.iterrows():
            width = row['max_y'] - row['min_y']
            height = row['max_z'] - row['min_z']
            rect = Rectangle((row['min_y'], row['min_z']), width, height,
                           linewidth=2, edgecolor='yellow', facecolor='none',
                           label='Wall BBox' if idx == 0 else '')
            ax.add_patch(rect)
            
            # Add wall ID
            center_y = row['center_y']
            center_z = row['center_z']
            ax.text(center_y, center_z, f"W{row['wall_id']}", 
                   ha='center', va='center', fontsize=8, color='black',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
    
    plt.colorbar(scatter, ax=ax, label='X Position (m)')
    ax.set_xlabel('Y (meters)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Height (meters)', fontsize=14, fontweight='bold')
    title = f'Final Wall Reconstruction - Front View (Y-Z)\n({len(points):,} points'
    if bboxes is not None:
        title += f', {len(bboxes)} walls)'
    else:
        title += ')'
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    front_view_path = vis_dir / "final_front_view.png"
    plt.savefig(front_view_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    
    logger.info(f"Saved front view: {front_view_path}")
    
    # Save point cloud statistics
    stats_path = vis_dir / "point_cloud_stats.txt"
    with open(stats_path, 'w') as f:
        f.write("Final Point Cloud Statistics\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"Total Points: {len(points):,}\n")
        f.write(f"Point Cloud File: {final_ply.name}\n")
        if bboxes is not None:
            f.write(f"Total Walls Detected: {len(bboxes)}\n")
        f.write("\n")
        
        f.write("Bounding Box:\n")
        f.write(f"  X: [{x.min():.3f}, {x.max():.3f}] m (range: {x.max()-x.min():.3f} m)\n")
        f.write(f"  Y: [{y.min():.3f}, {y.max():.3f}] m (range: {y.max()-y.min():.3f} m)\n")
        f.write(f"  Z: [{z.min():.3f}, {z.max():.3f}] m (range: {z.max()-z.min():.3f} m)\n\n")
        
        f.write("Center Point:\n")
        f.write(f"  X: {x.mean():.3f} m\n")
        f.write(f"  Y: {y.mean():.3f} m\n")
        f.write(f"  Z: {z.mean():.3f} m\n")
        
        if bboxes is not None:
            f.write("\n")
            f.write("=" * 80 + "\n")
            f.write("Wall Bounding Boxes Summary\n")
            f.write("=" * 80 + "\n\n")
            for idx, row in bboxes.iterrows():
                f.write(f"Wall {int(row['wall_id'])}:\n")
                f.write(f"  Center: ({row['center_x']:.3f}, {row['center_y']:.3f}, {row['center_z']:.3f}) m\n")
                f.write(f"  Dimensions (X×Y×Z): {row['dx']:.3f} × {row['dy']:.3f} × {row['dz']:.3f} m\n")
                f.write(f"  Points: {int(row['n_points']):,}\n")
                f.write(f"  Source: {row['ply_file']}\n")
                f.write("\n")
    
    logger.info(f"Saved statistics: {stats_path}")
    
    logger.info("")
    logger.info("=" * 60)
    logger.info("Visualization complete!")
    logger.info(f"Visualizations saved to: {vis_dir}")
    logger.info("=" * 60)
    
    return vis_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run incremental corridor pipeline with batch or file watcher mode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Batch mode (process all files at once)
  python run_corridor_incremental.py --root /path/to/scripts --batches /path/to/pointclouds
  
  # File watcher mode (continuously monitor directory)
  python run_corridor_incremental.py --root /path/to/scripts --watch /path/to/pointclouds --interval 5
        """
    )
    parser.add_argument("--root", 
                        help="Root folder containing scripts (REQUIRED)",
                        required=True)
    parser.add_argument("--batches", 
                        help="Batch directory containing point cloud files (batch mode)")
    parser.add_argument("--watch", 
                        help="Directory to monitor for new files (file watcher mode)")
    parser.add_argument("--interval", 
                        type=float, 
                        default=5.0,
                        help="Poll interval in seconds for file watcher mode (default: 5.0)")
    
    args = parser.parse_args()

    # Validate arguments
    if args.batches and args.watch:
        parser.error("Cannot use both --batches and --watch. Choose one mode.")
    
    if not args.batches and not args.watch:
        parser.error("Must specify either --batches (batch mode) or --watch (file watcher mode)")

    root_path = Path(args.root)
    paths = build_paths(root_path)
    
    # Determine mode and set up paths
    if args.watch:
        # File watcher mode
        watch_directory = Path(args.watch)
        if not watch_directory.exists():
            logger.error(f"Watch directory does not exist: {watch_directory}")
            exit(1)
        
        paths["BATCH_DIR"] = watch_directory
        batch_folder_name = watch_directory.name
        paths["OUT_ROOT"] = root_path / f"output_{batch_folder_name}"
        
        logger.info(f"Using root: {paths['ROOT']}")
        logger.info(f"Watch directory: {paths['BATCH_DIR']}")
        logger.info(f"Output directory: {paths['OUT_ROOT']}")
        
        # Create output directory
        paths["OUT_ROOT"].mkdir(parents=True, exist_ok=True)
        
        # Initialize pipeline and run file watcher
        pipeline = WallReconstructionPipeline(paths)
        pipeline.run_file_watcher(watch_directory, args.interval)
        
    else:
        # Batch mode
        batch_directory = Path(args.batches)
        if not batch_directory.exists():
            logger.error(f"Batch directory does not exist: {batch_directory}")
            exit(1)
        
        paths["BATCH_DIR"] = batch_directory
        batch_folder_name = batch_directory.name
        paths["OUT_ROOT"] = root_path / f"output_{batch_folder_name}"
        
        logger.info(f"Using root: {paths['ROOT']}")
        logger.info(f"Batch directory: {paths['BATCH_DIR']}")
        logger.info(f"Output directory: {paths['OUT_ROOT']}")
        
        # Get all point cloud files (PLY and LAS)
        import re
        
        all_ply_files = list(batch_directory.glob("*.ply"))
        all_las_files = list(batch_directory.glob("*.las")) + list(batch_directory.glob("*.laz"))
        
        # Convert LAS files to PLY if found
        if all_las_files:
            if not LAS_SUPPORT:
                logger.error("Found LAS/LAZ files but laspy is not installed.")
                logger.error("Install with: pip install laspy")
                exit(1)
            
            logger.info(f"Found {len(all_las_files)} LAS/LAZ files to convert")
            converted_dir = batch_directory / "converted_ply"
            
            for las_file in all_las_files:
                try:
                    ply_file = convert_las_to_ply(las_file, converted_dir)
                    all_ply_files.append(ply_file)
                except Exception as e:
                    logger.error(f"Failed to convert {las_file.name}: {e}")
                    continue
        
        # Extract numbers from filenames and sort
        def extract_number(filepath):
            numbers = re.findall(r'\d+', filepath.stem)
            return int(numbers[0]) if numbers else 0
        
        batches = sorted(all_ply_files, key=extract_number)
        
        if not batches:
            logger.error(f"No .ply or .las/.laz files found in {batch_directory}")
            exit(1)
        
        # Create output directory
        paths["OUT_ROOT"].mkdir(parents=True, exist_ok=True)
        
        # Initialize pipeline and run batch mode
        pipeline = WallReconstructionPipeline(paths)
        pipeline.run_batch_mode(batches)
