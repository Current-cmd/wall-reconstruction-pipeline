#!/usr/bin/env python3
import argparse
import os
import numpy as np
import open3d as o3d


def estimate_normals(pcd, k=30):
    # Fast normal estimation: no expensive global consistency
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k)
    )
    # We don’t need orient_normals_consistent_tangent_plane:
    # we only use abs(dot(normal, up)), so the sign doesn’t matter.


def infer_up_vector(pcd, up_axis, plane_dist, plane_deg):
    """Return the 'up' unit vector."""
    if up_axis in ("x", "y", "z"):
        mapping = {
            "x": np.array([1.0, 0.0, 0.0]),
            "y": np.array([0.0, 1.0, 0.0]),
            "z": np.array([0.0, 0.0, 1.0]),
        }
        return mapping[up_axis]
    elif up_axis == "auto":
        # Largest plane normal = up
        model, inliers = pcd.segment_plane(
            distance_threshold=plane_dist,
            ransac_n=3,
            num_iterations=3000,
        )
        n = np.array(model[:3], dtype=float)
        n /= np.linalg.norm(n)
        print(f"Auto-UP from largest plane: {n}  (inliers ~{len(inliers)})")
        return n
    else:
        raise ValueError(f"Unsupported up-axis: {up_axis}")


def keep_vertical_auto_strict(
    input_path,
    output_path,
    voxel_size=0.0,
    up_axis="z",
    plane_dist=0.04,
    plane_deg=8.0,
    vertical_deg=6.0,
    k=50,
    nb_neighbors=50,
    std_ratio=1.0,
    height_margin=0.0,
):
    """
    Simple strict vertical-wall filter:

    1) estimate normals
    2) remove horizontals: normal within plane_deg of 'up'
    3) keep verticals: normal within vertical_deg of 90° from 'up'
    4) run statistical outlier removal
    """
    print(f"[INFO] Loading: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    if voxel_size > 0:
        print(f"[INFO] Voxel downsample: {voxel_size}")
        pcd = pcd.voxel_down_sample(voxel_size)

    if len(pcd.points) == 0:
        raise RuntimeError("Input point cloud is empty")

    # 1) normals + up vector
    estimate_normals(pcd, k=k)
    up = infer_up_vector(pcd, up_axis, plane_dist, plane_deg)
    up = up / np.linalg.norm(up)

    normals = np.asarray(pcd.normals)
    pts = np.asarray(pcd.points)
    cols = np.asarray(pcd.colors) if pcd.has_colors() else None

    # 2) remove horizontals: angle(normal, up) <= plane_deg
    dot = np.abs(normals @ up)
    horiz_mask = dot >= np.cos(np.deg2rad(plane_deg))
    keep_mask = ~horiz_mask

    pts1 = pts[keep_mask]
    cols1 = cols[keep_mask] if cols is not None else None
    print(f"[INFO] Removed horizontals: {np.count_nonzero(horiz_mask)} points")

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pts1)
    if cols1 is not None:
        pcd1.colors = o3d.utility.Vector3dVector(cols1)

    # 3) recompute normals and keep strict verticals
    estimate_normals(pcd1, k=k)
    normals1 = np.asarray(pcd1.normals)
    pts1 = np.asarray(pcd1.points)
    cols1 = np.asarray(pcd1.colors) if pcd1.has_colors() else None

    dot1 = np.abs(normals1 @ up)
    vertical_thresh = np.sin(np.deg2rad(vertical_deg))  # small dot = ~90°
    vert_mask = dot1 <= vertical_thresh

    pts2 = pts1[vert_mask]
    cols2 = cols1[vert_mask] if cols1 is not None else None
    print(f"[INFO] Kept verticals: {np.count_nonzero(vert_mask)} points")

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pts2)
    if cols2 is not None:
        pcd2.colors = o3d.utility.Vector3dVector(cols2)

    # Optional 3b) crop off floor & ceiling band along the up-axis
    if height_margin > 0.0 and len(pts2) > 0:
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_idx = axis_map.get(up_axis, 2)
        coords = pts2[:, axis_idx]
        cmin, cmax = coords.min(), coords.max()
        mask_h = (coords >= cmin + height_margin) & (coords <= cmax - height_margin)
        removed = len(coords) - int(mask_h.sum())
        if removed > 0:
            print(f"[INFO] Height crop (margin={height_margin}): removed {removed} points")
            pts2 = pts2[mask_h]
            if cols2 is not None:
                cols2 = cols2[mask_h]

        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(pts2)
        if cols2 is not None:
            pcd2.colors = o3d.utility.Vector3dVector(cols2)

    # 4) remove speckle via statistical outlier removal
    if len(pcd2.points) > nb_neighbors:
        pcd_clean, _ = pcd2.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio,
        )
    else:
        pcd_clean = pcd2

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    o3d.io.write_point_cloud(output_path, pcd_clean)
    print(f"[OK] Saved strict vertical-only walls to: {output_path}")
    print(f"[OK] Points kept: {len(pcd_clean.points)}")


def parse_args():
    import argparse

    p = argparse.ArgumentParser(
        description="Keep only strict vertical walls from a point cloud."
    )
    p.add_argument("input", help="Input point cloud (.ply)")
    p.add_argument(
        "--out",
        required=True,
        help="Output PLY file for strict vertical walls",
    )
    p.add_argument(
        "--voxel",
        type=float,
        default=0.0,
        help="Voxel size for optional downsampling (0 = no downsample)",
    )
    p.add_argument(
        "--up-axis",
        type=str,
        default="z",
        choices=["x", "y", "z"],
        help="Axis that corresponds to 'up' in the point cloud",
    )
    p.add_argument(
        "--plane-dist",
        type=float,
        default=0.04,
        help="Distance threshold for RANSAC plane removal (unused if 0)",
    )
    p.add_argument(
        "--plane-deg",
        type=float,
        default=8.0,
        help="Max angle (deg) between normal and up-axis to treat as horizontal",
    )
    p.add_argument(
        "--vertical-deg",
        type=float,
        default=6.0,
        help="Angle tolerance (deg) around 90° for vertical classification",
    )
    p.add_argument(
        "--k",
        type=int,
        default=50,
        help="k-NN used for normal estimation",
    )
    p.add_argument(
        "--nb-neighbors",
        type=int,
        default=50,
        help="Neighbors for statistical outlier removal",
    )
    p.add_argument(
        "--std-ratio",
        type=float,
        default=1.0,
        help="Std dev multiplier for statistical outlier removal",
    )
    p.add_argument(
        "--height-margin",
        type=float,
        default=0.0,
        help=(
            "Crop off this margin (in up-axis units) from floor & ceiling "
            "(0 = no cropping)"
        ),
    )
    return p.parse_args()


def main():
    args = parse_args()
    keep_vertical_auto_strict(
        args.input,
        args.out,
        voxel_size=args.voxel,
        up_axis=args.up_axis,
        plane_dist=args.plane_dist,
        plane_deg=args.plane_deg,
        vertical_deg=args.vertical_deg,
        k=args.k,
        nb_neighbors=args.nb_neighbors,
        std_ratio=args.std_ratio,
        height_margin=args.height_margin,
    )


if __name__ == "__main__":
    main()
