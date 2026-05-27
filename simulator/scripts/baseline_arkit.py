#!/usr/bin/env python3
"""
ARKit Baseline Evaluation
Reconstructs point cloud using ARKit aligned_poses + depth,
then evaluates map quality against FARO GT mesh.

ARKit trajectory ATE = 0 by definition (aligned_poses IS the GT reference).
The value is in MAP QUALITY comparison: ARKit recon vs DV-SLAM recon vs FARO.

Usage:
    python baseline_arkit.py <scene_path> --output ./results/scene_id/arkit
"""

import sys
import os
import json
import glob
import numpy as np
from pathlib import Path

def load_scene(scene_path):
    """Load ScanNet++ iPhone data"""
    iphone_dir = os.path.join(scene_path, 'iphone')
    json_path = os.path.join(iphone_dir, 'pose_intrinsic_imu.json')

    with open(json_path) as f:
        data = json.load(f)

    frames = []
    for frame_name, frame_data in sorted(data.items()):
        if 'aligned_pose' not in frame_data or 'intrinsic' not in frame_data:
            continue
        frames.append({
            'name': frame_name,
            'timestamp': frame_data.get('timestamp', 0),
            'pose': np.array(frame_data['aligned_pose']),
            'raw_pose': np.array(frame_data.get('pose', frame_data['aligned_pose'])),
            'intrinsic': np.array(frame_data['intrinsic']),
        })

    return frames

def load_depth(depth_path):
    """Load 16-bit PNG depth (mm)"""
    try:
        import cv2
        depth = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
        if depth is None:
            return None
        return depth.astype(np.float32) / 1000.0  # mm → m
    except ImportError:
        from PIL import Image
        img = Image.open(depth_path)
        depth = np.array(img, dtype=np.float32) / 1000.0
        return depth

def unproject_depth(depth, K_rgb, pose, stride=4):
    """Unproject depth to 3D world points using pose"""
    h, w = depth.shape
    # Scale intrinsics from RGB (1920×1440) to depth (256×192)
    sx = w / 1920.0
    sy = h / 1440.0
    fx = K_rgb[0, 0] * sx
    fy = K_rgb[1, 1] * sy
    cx = K_rgb[0, 2] * sx
    cy = K_rgb[1, 2] * sy

    R = pose[:3, :3]
    t = pose[:3, 3]

    points = []
    for row in range(0, h, stride):
        for col in range(0, w, stride):
            d = depth[row, col]
            if d <= 0 or d > 10.0 or np.isnan(d):
                continue
            # Camera frame (ARKit: Y-up, Z-backward)
            x = (col - cx) * d / fx
            y = -((row - cy) * d / fy)
            z = -d
            p_cam = np.array([x, y, z])
            p_world = R @ p_cam + t
            points.append(p_world)

    return np.array(points) if points else np.zeros((0, 3))

def write_ply(path, points):
    """Write PLY file"""
    with open(path, 'wb') as f:
        header = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\n"
        header += "property float x\nproperty float y\nproperty float z\n"
        header += "end_header\n"
        f.write(header.encode())
        points.astype(np.float32).tofile(f)
    print(f"  PLY: {len(points)} pts → {path}")

def write_tum(path, frames):
    """Write TUM trajectory"""
    from scipy.spatial.transform import Rotation
    with open(path, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for fr in frames:
            t = fr['pose'][:3, 3]
            q = Rotation.from_matrix(fr['pose'][:3, :3]).as_quat()  # [x,y,z,w]
            f.write(f"{fr['timestamp']:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='ARKit Baseline Evaluation')
    parser.add_argument('scene_path', help='ScanNet++ scene directory')
    parser.add_argument('--output', default='./results/arkit', help='Output directory')
    parser.add_argument('--stride', type=int, default=4, help='Depth subsampling stride')
    parser.add_argument('--max-frames', type=int, default=0, help='Max frames (0=all)')
    args = parser.parse_args()

    scene_path = args.scene_path
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)

    print("╔══════════════════════════════════════╗")
    print("║   ARKit Baseline Reconstruction      ║")
    print("╚══════════════════════════════════════╝")

    # Load scene
    frames = load_scene(scene_path)
    print(f"Loaded {len(frames)} frames")

    depth_dir = os.path.join(scene_path, 'iphone', 'depth')
    if not os.path.exists(depth_dir):
        print(f"ERROR: depth directory not found: {depth_dir}")
        sys.exit(1)

    # Reconstruct
    all_points = []
    n = len(frames)
    if args.max_frames > 0:
        n = min(n, args.max_frames)

    for i, fr in enumerate(frames[:n]):
        depth_path = os.path.join(depth_dir, f"{fr['name']}.png")
        if not os.path.exists(depth_path):
            continue

        depth = load_depth(depth_path)
        if depth is None:
            continue

        pts = unproject_depth(depth, fr['intrinsic'], fr['pose'], stride=args.stride)
        all_points.append(pts)

        if (i + 1) % 50 == 0:
            total = sum(len(p) for p in all_points)
            print(f"  {i+1}/{n} frames, {total} pts total")

    if not all_points:
        print("ERROR: No points reconstructed")
        sys.exit(1)

    all_points = np.vstack(all_points)
    print(f"\nTotal: {len(all_points)} points")

    # Write outputs
    write_ply(os.path.join(output_dir, 'reconstruction.ply'), all_points)
    write_tum(os.path.join(output_dir, 'arkit_trajectory.txt'), frames[:n])

    # Write report
    with open(os.path.join(output_dir, 'report.txt'), 'w') as f:
        f.write("ARKit Baseline Report\n")
        f.write("=====================\n")
        f.write(f"Frames: {n}\n")
        f.write(f"Points: {len(all_points)}\n")
        f.write(f"Stride: {args.stride}\n")
        f.write(f"ATE RMSE: 0.0000 m (aligned_poses = GT reference)\n")
        f.write(f"Note: Evaluate map quality with CloudCompare vs FARO mesh\n")

    print(f"\n✅ Output: {output_dir}")

if __name__ == '__main__':
    main()
