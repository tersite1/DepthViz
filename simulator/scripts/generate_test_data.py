#!/usr/bin/env python3
"""Generate synthetic ScanNet++ test data for simulator validation."""

import os
import json
import numpy as np
from pathlib import Path

def rotation_matrix_y(angle_deg):
    a = np.radians(angle_deg)
    return np.array([
        [np.cos(a), 0, np.sin(a)],
        [0, 1, 0],
        [-np.sin(a), 0, np.cos(a)]
    ])

def main():
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'test_data', 'test_scene')
    iphone_dir = os.path.join(out_dir, 'iphone')
    depth_dir = os.path.join(iphone_dir, 'depth')
    scans_dir = os.path.join(out_dir, 'scans')
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(scans_dir, exist_ok=True)

    # Camera intrinsics (typical iPhone)
    fx_rgb, fy_rgb = 1400.0, 1400.0
    cx_rgb, cy_rgb = 960.0, 720.0
    K = [[fx_rgb, 0, cx_rgb], [0, fy_rgb, cy_rgb], [0, 0, 1]]

    # Generate 60 frames (2 seconds at 30fps)
    n_frames = 60
    data = {}

    for i in range(n_frames):
        frame_name = f"frame_{i:06d}"
        t = i / 30.0  # 30fps

        # Simulate camera moving forward + rotating
        angle = i * 2.0  # 2 deg/frame
        R = rotation_matrix_y(angle)
        pos = np.array([0.01 * i, 0.0, -0.005 * i])  # moving forward+right

        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = pos

        # IMU: gravity + small rotation
        gravity = [0.0, -1.0, 0.0]  # in g units
        user_accel = [0.001 * np.sin(t), 0.0, 0.001 * np.cos(t)]
        rot_rate = [0.0, np.radians(2.0 * 30), 0.0]  # 2 deg/frame at 30fps
        imu = list(rot_rate) + list(user_accel) + [0,0,0] + [0,0,0] + list(gravity)

        data[frame_name] = {
            "timestamp": t,
            "pose": pose.tolist(),
            "aligned_pose": pose.tolist(),
            "intrinsic": K,
            "imu": imu
        }

        # Generate synthetic depth (flat wall at 2m)
        w, h = 256, 192
        depth_mm = np.zeros((h, w), dtype=np.uint16)
        for r in range(h):
            for c in range(w):
                # Flat wall at z=2m with some noise
                base_depth = 2000  # mm
                noise = int(np.random.normal(0, 5))  # 5mm noise
                depth_mm[r, c] = max(0, base_depth + noise)

        # Save as 16-bit PNG
        try:
            import cv2
            cv2.imwrite(os.path.join(depth_dir, f"{frame_name}.png"), depth_mm)
        except ImportError:
            from PIL import Image
            img = Image.fromarray(depth_mm)
            img.save(os.path.join(depth_dir, f"{frame_name}.png"))

    # Save JSON
    json_path = os.path.join(iphone_dir, 'pose_intrinsic_imu.json')
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)

    # Create dummy GT mesh (just a few vertices)
    mesh_path = os.path.join(scans_dir, 'mesh_aligned_0.05.ply')
    with open(mesh_path, 'w') as f:
        f.write("ply\nformat ascii 1.0\nelement vertex 4\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        f.write("0 0 -2\n1 0 -2\n0 1 -2\n1 1 -2\n")

    print(f"Test data generated: {out_dir}")
    print(f"  {n_frames} frames, 256x192 depth, flat wall at 2m")
    print(f"  JSON: {json_path}")
    print(f"  Depth: {depth_dir}/")

if __name__ == '__main__':
    main()
