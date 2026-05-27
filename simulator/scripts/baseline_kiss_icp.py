#!/usr/bin/env python3
"""
KISS-ICP Baseline on ScanNet++ depth data.
Unprojected dToF point clouds → KISS-ICP odometry → TUM trajectory + PLY.

Usage:
    python baseline_kiss_icp.py <scene_path> --output ./results/kiss_icp

Requirements:
    pip install kiss-icp numpy
"""

import sys
import os
import json
import numpy as np
from pathlib import Path

def load_depth(path):
    try:
        import cv2
        d = cv2.imread(path, cv2.IMREAD_ANYDEPTH)
        return d.astype(np.float32) / 1000.0 if d is not None else None
    except ImportError:
        from PIL import Image
        return np.array(Image.open(path), dtype=np.float32) / 1000.0

def unproject(depth, fx, fy, cx, cy, stride=4):
    h, w = depth.shape
    pts = []
    for r in range(0, h, stride):
        for c in range(0, w, stride):
            d = depth[r, c]
            if d <= 0 or d > 10 or np.isnan(d):
                continue
            x = (c - cx) * d / fx
            y = -((r - cy) * d / fy)
            z = -d
            pts.append([x, y, z])
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 3), dtype=np.float64)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('scene_path')
    parser.add_argument('--output', default='./results/kiss_icp')
    parser.add_argument('--stride', type=int, default=4)
    args = parser.parse_args()

    try:
        from kiss_icp.pipeline import OdometryPipeline
        from kiss_icp.config import KISSConfig
    except ImportError:
        print("ERROR: pip install kiss-icp")
        sys.exit(1)

    scene_path = args.scene_path
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)

    # Load scene data
    json_path = os.path.join(scene_path, 'iphone', 'pose_intrinsic_imu.json')
    with open(json_path) as f:
        data = json.load(f)

    frames = sorted(data.items())
    depth_dir = os.path.join(scene_path, 'iphone', 'depth')

    # Get intrinsics
    first_frame = frames[0][1]
    K = np.array(first_frame['intrinsic'])
    sx, sy = 256/1920, 192/1440
    fx, fy = K[0,0]*sx, K[1,1]*sy
    cx, cy = K[0,2]*sx, K[1,2]*sy

    print(f"KISS-ICP on {len(frames)} frames (fx={fx:.1f}, stride={args.stride})")

    # Collect point clouds
    point_clouds = []
    timestamps = []
    gt_poses = []

    for frame_name, frame_data in frames:
        depth_path = os.path.join(depth_dir, f"{frame_name}.png")
        if not os.path.exists(depth_path):
            continue
        depth = load_depth(depth_path)
        if depth is None:
            continue

        pts = unproject(depth, fx, fy, cx, cy, args.stride)
        if len(pts) < 10:
            continue

        point_clouds.append(pts)
        timestamps.append(frame_data.get('timestamp', len(timestamps)/30.0))
        if 'aligned_pose' in frame_data:
            gt_poses.append(np.array(frame_data['aligned_pose']))

    print(f"  {len(point_clouds)} valid frames, avg {np.mean([len(p) for p in point_clouds]):.0f} pts/frame")

    # Run KISS-ICP
    from kiss_icp.kiss_icp import KissICP
    config = KISSConfig()
    config.data.max_range = 10.0
    config.data.min_range = 0.1
    config.data.deskew = False  # ScanNet++ has no per-point timestamps
    config.mapping.voxel_size = 0.5
    odometry = KissICP(config=config)

    estimated_poses = []
    for i, cloud in enumerate(point_clouds):
        try:
            source, _ = odometry.register_frame(cloud, np.zeros(len(cloud)))
        except Exception as e:
            print(f"  ⚠️ Frame {i} failed: {e}")
            continue
        estimated_poses.append(odometry.last_pose.copy())
        if (i+1) % 50 == 0:
            print(f"  {i+1}/{len(point_clouds)}")

    # Write TUM trajectory
    tum_path = os.path.join(output_dir, 'estimated_trajectory.txt')
    gt_path = os.path.join(output_dir, 'gt_trajectory.txt')

    from scipy.spatial.transform import Rotation
    with open(tum_path, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i, pose in enumerate(estimated_poses):
            t = pose[:3, 3]
            q = Rotation.from_matrix(pose[:3, :3]).as_quat()
            ts = timestamps[i]
            f.write(f"{ts:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

    if gt_poses:
        with open(gt_path, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for i, pose in enumerate(gt_poses[:len(timestamps)]):
                t = pose[:3, 3]
                q = Rotation.from_matrix(pose[:3, :3]).as_quat()
                ts = timestamps[i]
                f.write(f"{ts:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                        f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

    print(f"  ✅ KISS-ICP: {len(estimated_poses)} poses → {output_dir}")

if __name__ == '__main__':
    main()
