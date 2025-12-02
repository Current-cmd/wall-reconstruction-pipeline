#!/usr/bin/env python3
"""
Visualize RANSAC wall bounding boxes on point clouds for all outputs.
Creates 2D and 3D visualizations with bounding boxes overlaid on point clouds.
"""

import numpy as np
import open3d as o3d
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pathlib import Path
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def load_wall_metadata(json_path):
    """Load wall metadata from JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data.get('walls', [])


def create_bounding_box_lineset(wall):
    """Create Open3D LineSet for a wall's bounding box."""
    # Extract wall properties from metadata
    pos_2d = np.array(wall['position_2d'])
    endpoints = np.array(wall['boundary_endpoints'])
    thickness = wall['thickness']
    z_min = wall['z_min']
    z_max = wall['z_max']
    orientation = wall['orientation']
    
    # Calculate bounding box corners based on orientation
    if orientation == "X-aligned":
        # Wall extends along X axis
        x_min = min(endpoints[0][0], endpoints[1][0])
        x_max = max(endpoints[0][0], endpoints[1][0])
        y_center = pos_2d[1]
        y_min = y_center - thickness / 2
        y_max = y_center + thickness / 2
    else:  # Y-aligned
        # Wall extends along Y axis
        y_min = min(endpoints[0][1], endpoints[1][1])
        y_max = max(endpoints[0][1], endpoints[1][1])
        x_center = pos_2d[0]
        x_min = x_center - thickness / 2
        x_max = x_center + thickness / 2
    
    # Create 8 corners of the bounding box
    corners = np.array([
        [x_min, y_min, z_min],
        [x_max, y_min, z_min],
        [x_max, y_max, z_min],
        [x_min, y_max, z_min],
        [x_min, y_min, z_max],
        [x_max, y_min, z_max],
        [x_max, y_max, z_max],
        [x_min, y_max, z_max],
    ])
    
    # Define lines connecting corners
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
        [0, 4], [1, 5], [2, 6], [3, 7],  # Vertical edges
    ]
    
    # Create LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    # Color based on orientation
    if orientation == 'X-aligned':
        color = [0, 0, 1]  # Blue for X-aligned
    elif orientation == 'Y-aligned':
        color = [1, 0, 0]  # Red for Y-aligned
    else:
        color = [0, 1, 0]  # Green for unknown
    
    colors = [color for _ in range(len(lines))]
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set


def create_3d_visualization(pcd_path, walls, output_path):
    """Create 3D visualization with bounding boxes."""
    logger.info(f"Loading point cloud: {pcd_path}")
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    
    logger.info(f"Creating {len(walls)} bounding boxes")
    geometries = [pcd]
    
    for wall in walls:
        bbox_lineset = create_bounding_box_lineset(wall)
        geometries.append(bbox_lineset)
    
    # Save visualization
    logger.info(f"Saving 3D visualization: {output_path}")
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for geom in geometries:
        vis.add_geometry(geom)
    vis.update_renderer()
    vis.capture_screen_image(str(output_path), do_render=True)
    vis.destroy_window()
    
    return pcd, geometries


def create_2d_visualization(pcd, walls, output_path, view='top'):
    """Create 2D top-down or side view visualization with bounding boxes."""
    logger.info(f"Creating 2D {view} view visualization")
    
    # Get point cloud data
    points = np.asarray(pcd.points)
    
    # Determine axes based on view
    if view == 'top':
        x_idx, y_idx, z_idx = 0, 1, 2
        xlabel, ylabel = 'X (m)', 'Y (m)'
    elif view == 'front':
        x_idx, y_idx, z_idx = 0, 2, 1
        xlabel, ylabel = 'X (m)', 'Z (m)'
    elif view == 'side':
        x_idx, y_idx, z_idx = 1, 2, 0
        xlabel, ylabel = 'Y (m)', 'Z (m)'
    else:
        raise ValueError(f"Unknown view: {view}")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(16, 12), dpi=150)
    
    # Plot point cloud (2D projection)
    ax.scatter(points[:, x_idx], points[:, y_idx], c='gray', s=0.1, alpha=0.3, label='Point Cloud')
    
    # Plot bounding boxes
    for i, wall in enumerate(walls):
        pos_2d = np.array(wall['position_2d'])
        endpoints = np.array(wall['boundary_endpoints'])
        thickness = wall['thickness']
        z_min = wall['z_min']
        z_max = wall['z_max']
        orientation = wall['orientation']
        
        # Determine color
        if orientation == 'X-aligned':
            color = 'blue'
            label = 'X-aligned' if i == 0 else None
        elif orientation == 'Y-aligned':
            color = 'red'
            label = 'Y-aligned' if i == 0 else None
        else:
            color = 'green'
            label = 'Unknown' if i == 0 else None
        
        # Calculate bounding box for the current view
        if view == 'top':
            if orientation == "X-aligned":
                x_min = min(endpoints[0][0], endpoints[1][0])
                x_max = max(endpoints[0][0], endpoints[1][0])
                y_min = pos_2d[1] - thickness / 2
                y_max = pos_2d[1] + thickness / 2
            else:  # Y-aligned
                y_min = min(endpoints[0][1], endpoints[1][1])
                y_max = max(endpoints[0][1], endpoints[1][1])
                x_min = pos_2d[0] - thickness / 2
                x_max = pos_2d[0] + thickness / 2
        elif view == 'front':
            if orientation == "X-aligned":
                x_min = min(endpoints[0][0], endpoints[1][0])
                x_max = max(endpoints[0][0], endpoints[1][0])
            else:  # Y-aligned
                x_min = pos_2d[0] - thickness / 2
                x_max = pos_2d[0] + thickness / 2
            y_min = z_min
            y_max = z_max
        else:  # side view
            if orientation == "X-aligned":
                x_min = pos_2d[1] - thickness / 2
                x_max = pos_2d[1] + thickness / 2
            else:  # Y-aligned
                x_min = min(endpoints[0][1], endpoints[1][1])
                x_max = max(endpoints[0][1], endpoints[1][1])
            y_min = z_min
            y_max = z_max
        
        width = x_max - x_min
        height = y_max - y_min
        
        rect = Rectangle((x_min, y_min), width, height, 
                        linewidth=2, edgecolor=color, facecolor='none', 
                        label=label)
        ax.add_patch(rect)
        
        # Add wall ID at center
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        ax.text(center_x, center_y, f"W{wall['wall_id']}", 
               ha='center', va='center', fontsize=8, 
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
    
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(f'{view.capitalize()} View - Wall Bounding Boxes', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    # Add legend (remove duplicates)
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    logger.info(f"Saved 2D visualization: {output_path}")


def create_wall_heatmap(pcd, walls, output_path):
    """Create 2D heatmap showing wall density."""
    logger.info("Creating wall density heatmap")
    
    points = np.asarray(pcd.points)
    
    fig, ax = plt.subplots(figsize=(16, 12), dpi=150)
    
    # Create 2D histogram (heatmap)
    x_points = points[:, 0]
    y_points = points[:, 1]
    
    hist, xedges, yedges = np.histogram2d(x_points, y_points, bins=200)
    
    # Plot heatmap
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
    im = ax.imshow(hist.T, extent=extent, origin='lower', cmap='viridis', 
                   aspect='auto', alpha=0.6, interpolation='bilinear')
    
    # Overlay bounding boxes
    for wall in walls:
        pos_2d = np.array(wall['position_2d'])
        endpoints = np.array(wall['boundary_endpoints'])
        thickness = wall['thickness']
        orientation = wall['orientation']
        
        color = 'cyan' if orientation == 'X-aligned' else 'yellow'
        
        if orientation == "X-aligned":
            x_min = min(endpoints[0][0], endpoints[1][0])
            x_max = max(endpoints[0][0], endpoints[1][0])
            y_min = pos_2d[1] - thickness / 2
            y_max = pos_2d[1] + thickness / 2
        else:  # Y-aligned
            y_min = min(endpoints[0][1], endpoints[1][1])
            y_max = max(endpoints[0][1], endpoints[1][1])
            x_min = pos_2d[0] - thickness / 2
            x_max = pos_2d[0] + thickness / 2
        
        width = x_max - x_min
        height = y_max - y_min
        
        rect = Rectangle((x_min, y_min), width, height, 
                        linewidth=2.5, edgecolor=color, facecolor='none')
        ax.add_patch(rect)
        
        # Add wall ID
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        ax.text(center_x, center_y, f"W{wall['wall_id']}", 
               ha='center', va='center', fontsize=9, color='white',
               bbox=dict(boxstyle='round,pad=0.4', facecolor='black', alpha=0.7))
    
    plt.colorbar(im, ax=ax, label='Point Density')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Wall Detection Heatmap', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, color='white')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    logger.info(f"Saved heatmap: {output_path}")


def create_wall_statistics_plot(walls, output_path):
    """Create statistical visualization of wall properties."""
    logger.info("Creating wall statistics plot")
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Extract data
    wall_ids = [w['wall_id'] for w in walls]
    heights = [w['height'] for w in walls]
    thicknesses = [w['thickness'] for w in walls]
    orientations = [w['orientation'] for w in walls]
    lengths = [w['length'] for w in walls]
    
    # Calculate areas (length × height)
    areas = [l * h for l, h in zip(lengths, heights)]
    
    # Plot 1: Wall heights
    ax1 = axes[0, 0]
    colors_height = ['blue' if o == 'X-aligned' else 'red' for o in orientations]
    ax1.bar(wall_ids, heights, color=colors_height, alpha=0.7)
    ax1.set_xlabel('Wall ID')
    ax1.set_ylabel('Height (m)')
    ax1.set_title('Wall Heights')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Wall thicknesses
    ax2 = axes[0, 1]
    ax2.bar(wall_ids, thicknesses, color=colors_height, alpha=0.7)
    ax2.set_xlabel('Wall ID')
    ax2.set_ylabel('Thickness (m)')
    ax2.set_title('Wall Thicknesses')
    ax2.axhline(y=0.15, color='green', linestyle='--', label='Min (0.15m)')
    ax2.axhline(y=0.50, color='orange', linestyle='--', label='Max (0.50m)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Wall areas
    ax3 = axes[1, 0]
    ax3.bar(wall_ids, areas, color=colors_height, alpha=0.7)
    ax3.set_xlabel('Wall ID')
    ax3.set_ylabel('Area (m²)')
    ax3.set_title('Wall Surface Areas')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Orientation distribution
    ax4 = axes[1, 1]
    orientation_counts = {}
    for o in orientations:
        orientation_counts[o] = orientation_counts.get(o, 0) + 1
    
    colors_pie = {'X-aligned': 'blue', 'Y-aligned': 'red', 'unknown': 'green'}
    pie_colors = [colors_pie.get(o, 'gray') for o in orientation_counts.keys()]
    
    ax4.pie(orientation_counts.values(), labels=orientation_counts.keys(), 
           autopct='%1.1f%%', colors=pie_colors, startangle=90)
    ax4.set_title('Wall Orientation Distribution')
    
    plt.suptitle(f'Wall Statistics Summary ({len(walls)} walls)', 
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    logger.info(f"Saved statistics plot: {output_path}")


def process_output_directory(output_dir):
    """Process a single output directory and create all visualizations."""
    output_path = Path(output_dir)
    
    logger.info(f"\n{'='*80}")
    logger.info(f"Processing: {output_path.name}")
    logger.info(f"{'='*80}")
    
    # Find the latest iteration files
    pcd_dir = output_path / "point_clouds"
    metadata_dir = output_path / "metadata"
    
    # Get all point cloud files and find the last one
    pcd_files = sorted(list(pcd_dir.glob("global_vert_iter_*.ply")))
    if not pcd_files:
        logger.error(f"No point cloud files found in: {pcd_dir}")
        return
    
    pcd_path = pcd_files[-1]
    iter_num = pcd_path.stem.split('_')[-1]
    
    # Use matching metadata file (try both iter_XXXX and iter_9999)
    metadata_path = metadata_dir / f"walls_iter_{iter_num}.json"
    if not metadata_path.exists():
        metadata_path = metadata_dir / "walls_iter_9999.json"
    
    bbox_dir = output_path / "bounding_boxes"
    
    logger.info(f"Using point cloud: {pcd_path.name}")
    logger.info(f"Using metadata: {metadata_path.name}")
    
    # Check if files exist
    if not metadata_path.exists():
        logger.error(f"Metadata not found: {metadata_path}")
        return
    
    if not pcd_path.exists():
        logger.error(f"Point cloud not found: {pcd_path}")
        return
    
    # Create bounding_boxes directory
    bbox_dir.mkdir(exist_ok=True)
    logger.info(f"Created directory: {bbox_dir}")
    
    # Load wall metadata
    walls = load_wall_metadata(metadata_path)
    logger.info(f"Loaded {len(walls)} walls")
    
    # Create visualizations
    logger.info("\n--- Creating 3D Visualization ---")
    pcd, geometries = create_3d_visualization(
        pcd_path, walls, bbox_dir / "3d_bounding_boxes.png"
    )
    
    logger.info("\n--- Creating 2D Top View ---")
    create_2d_visualization(
        pcd, walls, bbox_dir / "2d_top_view.png", view='top'
    )
    
    logger.info("\n--- Creating 2D Front View ---")
    create_2d_visualization(
        pcd, walls, bbox_dir / "2d_front_view.png", view='front'
    )
    
    logger.info("\n--- Creating 2D Side View ---")
    create_2d_visualization(
        pcd, walls, bbox_dir / "2d_side_view.png", view='side'
    )
    
    logger.info("\n--- Creating Heatmap ---")
    create_wall_heatmap(
        pcd, walls, bbox_dir / "wall_heatmap.png"
    )
    
    logger.info("\n--- Creating Statistics Plot ---")
    create_wall_statistics_plot(
        walls, bbox_dir / "wall_statistics.png"
    )
    
    # Save wall metadata summary
    summary_path = bbox_dir / "wall_summary.txt"
    with open(summary_path, 'w') as f:
        f.write(f"Wall Bounding Box Summary - {output_path.name}\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"Total Walls: {len(walls)}\n\n")
        
        for wall in walls:
            f.write(f"Wall {wall['wall_id']}:\n")
            f.write(f"  Orientation: {wall['orientation']}\n")
            f.write(f"  Position 2D: {wall['position_2d']}\n")
            f.write(f"  Length: {wall['length']:.3f}m\n")
            f.write(f"  Thickness: {wall['thickness']:.3f}m\n")
            f.write(f"  Height: {wall['height']:.3f}m\n")
            f.write(f"  Z Range: {wall['z_min']:.3f}m to {wall['z_max']:.3f}m\n")
            f.write(f"  Points: {wall['num_points']}\n")
            f.write(f"  Endpoints: {wall['boundary_endpoints']}\n")
            f.write("\n")
    
    logger.info(f"Saved summary: {summary_path}")
    logger.info(f"\n✅ Completed processing: {output_path.name}\n")


def process_ai_walls(ply_path, output_dir):
    """Process AI-segmented wall point cloud and create visualizations."""
    logger.info(f"\n{'='*80}")
    logger.info(f"Processing AI Walls: {ply_path.name}")
    logger.info(f"{'='*80}")
    
    # Load point cloud
    logger.info(f"Loading point cloud: {ply_path}")
    pcd = o3d.io.read_point_cloud(str(ply_path))
    points = np.asarray(pcd.points)
    
    logger.info(f"Total points: {len(points):,}")
    
    # Create bounding boxes directory
    bbox_dir = output_dir / "bounding_boxes"
    bbox_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Created directory: {bbox_dir}")
    
    # Create simple 2D top view with wall outlines
    logger.info("\n--- Creating 2D Top View ---")
    fig, ax = plt.subplots(figsize=(20, 16), dpi=150)
    
    x = points[:, 0]
    y = points[:, 1]
    
    ax.scatter(x, y, c='#2E86AB', s=0.3, alpha=0.6, label='Wall Points')
    
    ax.set_xlabel('X (meters)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (meters)', fontsize=14, fontweight='bold')
    ax.set_title(f'2D Top View - {ply_path.stem}', fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='upper right', fontsize=12)
    
    plt.tight_layout()
    output_path = bbox_dir / "2d_top_view.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    
    logger.info(f"Saved 2D visualization: {output_path}")
    logger.info(f"\n✅ Completed processing: {ply_path.name}\n")


def main():
    """Main function to process all output directories."""
    base_dir = Path(__file__).parent
    
    output_dirs = [
        base_dir / "output_clips",
        base_dir / "output_4th_floor",
        base_dir / "output_building2",
        base_dir / "output_ai_2nd_floor_ransac",
        base_dir / "output_ai_4th_floor_ransac"
    ]
    
    # AI wall files to process (raw point clouds without bounding boxes)
    ai_wall_files = [
        (base_dir / "walls_2nd_floor_ai.ply", base_dir / "output_ai_2nd_floor")
    ]
    
    logger.info("="*80)
    logger.info("RANSAC Bounding Box Visualization")
    logger.info("="*80)
    
    for output_dir in output_dirs:
        if output_dir.exists():
            try:
                process_output_directory(output_dir)
            except Exception as e:
                logger.error(f"Error processing {output_dir}: {e}", exc_info=True)
        else:
            logger.warning(f"Directory not found: {output_dir}")
    
    # Process AI wall files
    logger.info("\n" + "="*80)
    logger.info("AI WALL VISUALIZATION")
    logger.info("="*80)
    
    for ply_path, output_dir in ai_wall_files:
        if ply_path.exists():
            try:
                process_ai_walls(ply_path, output_dir)
            except Exception as e:
                logger.error(f"Error processing {ply_path}: {e}", exc_info=True)
        else:
            logger.warning(f"File not found: {ply_path}")
    
    logger.info("\n" + "="*80)
    logger.info("ALL VISUALIZATIONS COMPLETE")
    logger.info("="*80)
    logger.info("\nOutputs saved in:")
    for output_dir in output_dirs:
        bbox_dir = output_dir / "bounding_boxes"
        if bbox_dir.exists():
            logger.info(f"  • {bbox_dir}")
    for _, output_dir in ai_wall_files:
        bbox_dir = output_dir / "bounding_boxes"
        if bbox_dir.exists():
            logger.info(f"  • {bbox_dir}")


if __name__ == "__main__":
    main()
