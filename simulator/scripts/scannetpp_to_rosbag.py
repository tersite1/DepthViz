#!/usr/bin/env python3
"""
ScanNet++ → ROS2 Bag Converter (rosbags 0.11+ API)
Converts ScanNet++ iPhone data to ROS2 bag for FAST-LIO, FAST-LIVO2, RTAB-Map.

Topics created:
  /camera/depth       (sensor_msgs/msg/Image, 16UC1, mm)
  /camera/info        (sensor_msgs/msg/CameraInfo)
  /imu/data           (sensor_msgs/msg/Imu)
  /lidar/points       (sensor_msgs/msg/PointCloud2)

Usage:
    python scannetpp_to_rosbag.py <scene_path> --output scene_bag/

Requirements:
    pip install rosbags numpy opencv-python-headless
    (rosbags is a pure-Python ROS bag writer, no ROS install needed)
"""

import sys
import os
import json
import struct
import numpy as np
from pathlib import Path


def load_depth(path):
    try:
        import cv2
        d = cv2.imread(str(path), cv2.IMREAD_ANYDEPTH)
        return d if d is not None else None
    except ImportError:
        from PIL import Image
        return np.array(Image.open(path), dtype=np.uint16)


def unproject(depth_mm, fx, fy, cx, cy, stride=4):
    h, w = depth_mm.shape
    pts = []
    for r in range(0, h, stride):
        for c in range(0, w, stride):
            d_mm = float(depth_mm[r, c])
            if d_mm <= 0 or d_mm > 10000:
                continue
            d = d_mm / 1000.0
            x = (c - cx) * d / fx
            y = -((r - cy) * d / fy)
            z = -d
            pts.append([x, y, z])
    return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='ScanNet++ to ROS2 bag converter')
    parser.add_argument('scene_path', help='ScanNet++ scene directory')
    parser.add_argument('--output', default='scene_bag', help='Output bag directory')
    parser.add_argument('--stride', type=int, default=4, help='Depth subsampling for point cloud')
    args = parser.parse_args()

    scene_path = args.scene_path
    iphone_dir = os.path.join(scene_path, 'iphone')

    # Load pose_intrinsic_imu.json
    json_path = os.path.join(iphone_dir, 'pose_intrinsic_imu.json')
    print(f"Loading {json_path}...")
    with open(json_path) as f:
        data = json.load(f)

    depth_dir = os.path.join(iphone_dir, 'depth')

    frames = []
    for frame_name, frame_data in sorted(data.items()):
        frames.append({'name': frame_name, 'ts': frame_data.get('timestamp', len(frames) / 30.0), 'data': frame_data})
    frames.sort(key=lambda x: x['ts'])

    print(f"Frames: {len(frames)}")

    # Get intrinsics
    K = np.array(frames[0]['data']['intrinsic'])
    sx, sy = 256 / 1920, 192 / 1440
    fx, fy = K[0, 0] * sx, K[1, 1] * sy
    cx, cy = K[0, 2] * sx, K[1, 2] * sy
    print(f"Depth intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    # Setup rosbags typestore
    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag2 import Writer

    store = get_typestore(Stores.ROS2_HUMBLE)
    Header = store.types['std_msgs/msg/Header']
    Time = store.types['builtin_interfaces/msg/Time']
    Imu = store.types['sensor_msgs/msg/Imu']
    Image = store.types['sensor_msgs/msg/Image']
    CameraInfo = store.types['sensor_msgs/msg/CameraInfo']
    PointCloud2 = store.types['sensor_msgs/msg/PointCloud2']
    PointField = store.types['sensor_msgs/msg/PointField']
    Quaternion = store.types['geometry_msgs/msg/Quaternion']
    Vector3 = store.types['geometry_msgs/msg/Vector3']

    output_dir = args.output
    import shutil
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    with Writer(output_dir, version=9) as bag:
        conn_imu = bag.add_connection('/imu/data', 'sensor_msgs/msg/Imu', typestore=store)
        conn_depth = bag.add_connection('/camera/depth', 'sensor_msgs/msg/Image', typestore=store)
        conn_info = bag.add_connection('/camera/info', 'sensor_msgs/msg/CameraInfo', typestore=store)
        conn_pts = bag.add_connection('/lidar/points', 'sensor_msgs/msg/PointCloud2', typestore=store)

        for i, frame in enumerate(frames):
            ts_sec = frame['ts']
            ts_ns = int(ts_sec * 1e9)
            stamp = Time(sec=int(ts_sec), nanosec=int((ts_sec % 1) * 1e9))
            hdr = Header(stamp=stamp, frame_id='imu')

            # --- IMU ---
            imu_raw = frame['data'].get('imu', [])
            if imu_raw:
                imu_data = np.array(imu_raw, dtype=np.float64)
                if imu_data.ndim == 1 and len(imu_data) >= 6:
                    ang_vel = Vector3(x=float(imu_data[0]), y=float(imu_data[1]), z=float(imu_data[2]))
                    lin_acc = Vector3(x=float(imu_data[3]) * 9.81, y=float(imu_data[4]) * 9.81, z=float(imu_data[5]) * 9.81)
                else:
                    ang_vel = Vector3(x=0.0, y=0.0, z=0.0)
                    lin_acc = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                ang_vel = Vector3(x=0.0, y=0.0, z=0.0)
                lin_acc = Vector3(x=0.0, y=0.0, z=0.0)

            imu_msg = Imu(
                header=hdr,
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=np.full(9, -1.0),
                angular_velocity=ang_vel,
                angular_velocity_covariance=np.zeros(9),
                linear_acceleration=lin_acc,
                linear_acceleration_covariance=np.zeros(9),
            )
            bag.write(conn_imu, ts_ns, store.serialize_cdr(imu_msg, 'sensor_msgs/msg/Imu'))

            # --- Depth image ---
            depth_path = os.path.join(depth_dir, f"{frame['name']}.png")
            if os.path.exists(depth_path):
                depth_mm = load_depth(depth_path)
                if depth_mm is not None:
                    h, w = depth_mm.shape
                    depth_hdr = Header(stamp=stamp, frame_id='camera')
                    depth_msg = Image(
                        header=depth_hdr,
                        height=h, width=w,
                        encoding='16UC1',
                        is_bigendian=False,
                        step=w * 2,
                        data=depth_mm.astype(np.uint16).flatten().view(np.uint8),
                    )
                    bag.write(conn_depth, ts_ns, store.serialize_cdr(depth_msg, 'sensor_msgs/msg/Image'))

                    # --- CameraInfo ---
                    RegionOfInterest = store.types['sensor_msgs/msg/RegionOfInterest']
                    roi = RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
                    info_msg = CameraInfo(
                        header=depth_hdr,
                        height=h, width=w,
                        distortion_model='plumb_bob',
                        d=np.zeros(5),
                        k=np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=np.float64),
                        r=np.eye(3).flatten(),
                        p=np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0], dtype=np.float64),
                        binning_x=0, binning_y=0, roi=roi,
                    )
                    bag.write(conn_info, ts_ns, store.serialize_cdr(info_msg, 'sensor_msgs/msg/CameraInfo'))

                    # --- PointCloud2 from depth ---
                    pts = unproject(depth_mm, fx, fy, cx, cy, args.stride)
                    if len(pts) > 0:
                        pc_hdr = Header(stamp=stamp, frame_id='camera')
                        fields = [
                            PointField(name='x', offset=0, datatype=7, count=1),
                            PointField(name='y', offset=4, datatype=7, count=1),
                            PointField(name='z', offset=8, datatype=7, count=1),
                        ]
                        pc_data = pts.astype(np.float32).tobytes()
                        pc_msg = PointCloud2(
                            header=pc_hdr,
                            height=1, width=len(pts),
                            fields=fields,
                            is_bigendian=False,
                            point_step=12,
                            row_step=12 * len(pts),
                            data=np.frombuffer(pc_data, dtype=np.uint8),
                            is_dense=True,
                        )
                        bag.write(conn_pts, ts_ns, store.serialize_cdr(pc_msg, 'sensor_msgs/msg/PointCloud2'))

            if (i + 1) % 20 == 0:
                print(f"  {i+1}/{len(frames)} frames written")

    print(f"\n✅ ROS2 bag written to: {output_dir}")
    print(f"   Topics: /imu/data, /camera/depth, /camera/info, /lidar/points")
    print(f"   Frames: {len(frames)}")


if __name__ == '__main__':
    main()
