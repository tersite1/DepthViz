#!/usr/bin/env python3
"""
RTAB-Map Baseline on ScanNet++ data.
Uses RTAB-Map's offline processing mode with RGB-D input.

Usage:
    python baseline_rtabmap.py <scene_path> --output ./results/rtabmap

Requirements:
    - rtabmap CLI installed (apt install rtabmap / brew install rtabmap)
    - OR: rtabmap-python (pip install rtabmap)
"""

import sys
import os
import json
import subprocess
import numpy as np
from pathlib import Path

def check_rtabmap():
    """Check if rtabmap is available"""
    try:
        result = subprocess.run(['rtabmap', '--version'], capture_output=True, text=True)
        return True
    except FileNotFoundError:
        return False

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('scene_path')
    parser.add_argument('--output', default='./results/rtabmap')
    args = parser.parse_args()

    scene_path = args.scene_path
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)

    iphone_dir = os.path.join(scene_path, 'iphone')
    depth_dir = os.path.join(iphone_dir, 'depth')
    rgb_dir = os.path.join(iphone_dir, 'rgb')

    if not os.path.exists(depth_dir) or not os.path.exists(rgb_dir):
        print("ERROR: Need both depth/ and rgb/ directories")
        print(f"  depth: {depth_dir} (exists: {os.path.exists(depth_dir)})")
        print(f"  rgb: {rgb_dir} (exists: {os.path.exists(rgb_dir)})")
        sys.exit(1)

    # Load intrinsics
    json_path = os.path.join(iphone_dir, 'pose_intrinsic_imu.json')
    with open(json_path) as f:
        data = json.load(f)

    first_frame = list(sorted(data.items()))[0][1]
    K = np.array(first_frame['intrinsic'])

    # Write camera calibration for RTAB-Map
    calib_path = os.path.join(output_dir, 'camera.yaml')
    with open(calib_path, 'w') as f:
        f.write(f"%YAML:1.0\n")
        f.write(f"image_width: 1920\n")
        f.write(f"image_height: 1440\n")
        f.write(f"camera_matrix: !!opencv-matrix\n")
        f.write(f"  rows: 3\n  cols: 3\n  dt: d\n")
        f.write(f"  data: [{K[0][0]}, 0, {K[0][2]}, 0, {K[1][1]}, {K[1][2]}, 0, 0, 1]\n")
        f.write(f"distortion_coefficients: !!opencv-matrix\n")
        f.write(f"  rows: 1\n  cols: 5\n  dt: d\n")
        f.write(f"  data: [0, 0, 0, 0, 0]\n")

    if check_rtabmap():
        print("RTAB-Map found! Running offline processing...")
        print("  Note: RTAB-Map processes RGB-D pairs sequentially")

        # RTAB-Map offline command
        db_path = os.path.join(output_dir, 'rtabmap.db')
        cmd = [
            'rtabmap-rgbd_mapping',
            '--Rtabmap/DetectionRate', '2',
            '--RGBD/LinearUpdate', '0.05',
            '--RGBD/AngularUpdate', '0.05',
            '--Mem/STMSize', '30',
            '--Kp/MaxFeatures', '400',
            '-d', db_path,
        ]
        print(f"  Command: {' '.join(cmd)}")
        print(f"  ⚠️  RTAB-Map offline mode requires specific data format.")
        print(f"  Use: rtabmap-reprocess {db_path}")
    else:
        print("RTAB-Map not found.")
        print("Install: sudo apt install rtabmap  (Linux)")
        print("         brew install rtabmap       (macOS)")
        print("\nAlternative: Use RTAB-Map iOS app on device,")
        print("then export trajectory for comparison.")

    # Write setup guide
    guide_path = os.path.join(output_dir, 'setup_guide.txt')
    with open(guide_path, 'w') as f:
        f.write("RTAB-Map Evaluation Guide\n")
        f.write("=" * 50 + "\n\n")
        f.write("Option 1: RTAB-Map CLI (offline)\n")
        f.write(f"  1. Install: sudo apt install rtabmap\n")
        f.write(f"  2. Create database:\n")
        f.write(f"     rtabmap-rgbd_dataset \\\n")
        f.write(f"       --Rtabmap/DetectionRate 2 \\\n")
        f.write(f"       {rgb_dir} {depth_dir} \\\n")
        f.write(f"       --output {output_dir}/rtabmap.db\n")
        f.write(f"  3. Export trajectory:\n")
        f.write(f"     rtabmap-export --poses {output_dir}/rtabmap.db\n\n")
        f.write("Option 2: ROS (rtabmap_ros)\n")
        f.write(f"  1. Convert to rosbag: python scannetpp_to_rosbag.py {scene_path}\n")
        f.write(f"  2. roslaunch rtabmap_ros rgbd_mapping.launch\n")
        f.write(f"  3. rosbag play scene.bag\n\n")
        f.write("Option 3: RTAB-Map iOS app\n")
        f.write(f"  1. Scan same environment on iPhone\n")
        f.write(f"  2. Export trajectory from app\n")
        f.write(f"  3. Compare with evo\n")

    print(f"\n  Guide written to: {guide_path}")

if __name__ == '__main__':
    main()
