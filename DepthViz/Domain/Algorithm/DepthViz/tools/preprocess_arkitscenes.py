#!/usr/bin/env python3
"""
Preprocess ARKitScenes sequences into the format expected by DV-SLAM's offline evaluator.

ARKitScenes raw format:
  lowres_depth/*.png           — uint16 PNG (depth in mm)
  confidence/*.png             — uint8 PNG (confidence 0/1/2)
  lowres_wide.traj             — timestamp ax ay az tx ty tz (axis-angle + translation)
  lowres_wide_intrinsics/*.pincam — width height fx fy cx cy (per-frame)

Output format:
  poses.txt                    — timestamp tx ty tz qx qy qz qw (TUM)
  intrinsics.txt               — fx fy cx cy width height
  depth/<timestamp>.bin        — float32 binary (depth in meters, row-major)
  confidence/<timestamp>.bin   — uint8 binary (confidence, row-major)

Usage:
  python3 preprocess_arkitscenes.py <arkitscenes_scene_dir> <output_dir>
"""

import sys
import os
import struct
import glob
import numpy as np
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow is required. Install with: pip3 install Pillow")
    sys.exit(1)

from scipy.spatial.transform import Rotation


def axis_angle_to_quaternion(ax, ay, az):
    """Convert axis-angle (radians) to quaternion (qx, qy, qz, qw)."""
    angle = np.sqrt(ax*ax + ay*ay + az*az)
    if angle < 1e-10:
        return 0.0, 0.0, 0.0, 1.0
    axis = np.array([ax, ay, az]) / angle
    qw = np.cos(angle / 2)
    s = np.sin(angle / 2)
    qx, qy, qz = axis * s
    return qx, qy, qz, qw


def load_traj(traj_path):
    """Load ARKitScenes .traj file → list of (timestamp, tx, ty, tz, qx, qy, qz, qw)."""
    poses = []
    with open(traj_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            ts = float(parts[0])
            ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
            tx, ty, tz = float(parts[4]), float(parts[5]), float(parts[6])
            qx, qy, qz, qw = axis_angle_to_quaternion(ax, ay, az)
            poses.append((ts, tx, ty, tz, qx, qy, qz, qw))
    return poses


def load_pincam(pincam_path):
    """Load .pincam intrinsics → (width, height, fx, fy, cx, cy)."""
    with open(pincam_path, 'r') as f:
        parts = f.read().strip().split()
    w, h = int(float(parts[0])), int(float(parts[1]))
    fx, fy = float(parts[2]), float(parts[3])
    cx, cy = float(parts[4]), float(parts[5])
    return w, h, fx, fy, cx, cy


def process_sequence(scene_dir, output_dir):
    scene_dir = Path(scene_dir)
    output_dir = Path(output_dir)

    # Find trajectory file
    traj_path = scene_dir / "lowres_wide.traj"
    if not traj_path.exists():
        print(f"ERROR: No trajectory file at {traj_path}")
        return False

    print(f"Loading trajectory: {traj_path}")
    poses = load_traj(traj_path)
    print(f"  Found {len(poses)} poses")

    if not poses:
        print("ERROR: No poses found")
        return False

    # Find depth maps
    depth_dir = scene_dir / "lowres_depth"
    if not depth_dir.exists():
        print(f"WARNING: No depth directory at {depth_dir}")
        depth_dir = None

    # Find confidence maps
    conf_dir = scene_dir / "confidence"
    if not conf_dir.exists():
        conf_dir = None
        print("WARNING: No confidence directory found")

    # Find intrinsics (use first .pincam file)
    intrinsics_dir = scene_dir / "lowres_wide_intrinsics"
    fx, fy, cx, cy, img_w, img_h = 500, 500, 128, 96, 256, 192
    if intrinsics_dir.exists():
        pincam_files = sorted(intrinsics_dir.glob("*.pincam"))
        if pincam_files:
            img_w, img_h, fx, fy, cx, cy = load_pincam(pincam_files[0])
            print(f"  Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} {img_w}x{img_h}")

    # Create output directories
    out_depth = output_dir / "depth"
    out_conf = output_dir / "confidence"
    out_depth.mkdir(parents=True, exist_ok=True)
    out_conf.mkdir(parents=True, exist_ok=True)

    # Write intrinsics
    with open(output_dir / "intrinsics.txt", 'w') as f:
        f.write(f"{fx} {fy} {cx} {cy} {img_w} {img_h}\n")

    # Write poses in TUM format
    with open(output_dir / "poses.txt", 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for ts, tx, ty, tz, qx, qy, qz, qw in poses:
            f.write(f"{ts:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    # Process depth and confidence maps
    processed = 0
    pose_timestamps = {p[0] for p in poses}

    if depth_dir:
        depth_files = sorted(depth_dir.glob("*.png"))
        print(f"  Found {len(depth_files)} depth maps")

        for depth_path in depth_files:
            # Extract timestamp from filename
            ts_str = depth_path.stem
            try:
                ts = float(ts_str)
            except ValueError:
                continue

            # Load depth PNG (uint16, millimeters)
            depth_img = np.array(Image.open(depth_path))
            if depth_img.dtype == np.uint16:
                depth_float = depth_img.astype(np.float32) / 1000.0  # mm → meters
            else:
                depth_float = depth_img.astype(np.float32)

            # Save as float32 binary
            out_path = out_depth / f"{ts:.6f}.bin"
            depth_float.tofile(str(out_path))

            # Load confidence if available
            if conf_dir:
                conf_path = conf_dir / depth_path.name
                if conf_path.exists():
                    conf_img = np.array(Image.open(conf_path)).astype(np.uint8)
                    conf_out = out_conf / f"{ts:.6f}.bin"
                    conf_img.tofile(str(conf_out))
                else:
                    # Generate default high confidence
                    conf_default = np.full(depth_float.shape, 2, dtype=np.uint8)
                    conf_default[depth_float <= 0] = 0
                    conf_out = out_conf / f"{ts:.6f}.bin"
                    conf_default.tofile(str(conf_out))
            else:
                # No confidence data — generate from depth validity
                conf_default = np.full(depth_float.shape, 2, dtype=np.uint8)
                conf_default[depth_float <= 0] = 0
                conf_out = out_conf / f"{ts:.6f}.bin"
                conf_default.tofile(str(conf_out))

            processed += 1

    print(f"  Processed {processed} depth frames")
    print(f"  Output: {output_dir}")
    return True


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 preprocess_arkitscenes.py <scene_dir> <output_dir>")
        print()
        print("Example:")
        print("  python3 preprocess_arkitscenes.py ~/ARKitScenes/raw/Training/47331606 ./data/47331606")
        sys.exit(1)

    scene_dir = sys.argv[1]
    output_dir = sys.argv[2]

    if not os.path.isdir(scene_dir):
        print(f"ERROR: Scene directory not found: {scene_dir}")
        sys.exit(1)

    success = process_sequence(scene_dir, output_dir)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
