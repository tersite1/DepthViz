# DV-SLAM ScanNet++ Simulator

Offline evaluation tool for DV-SLAM on ScanNet++ dataset.
Self-contained C++ pipeline — same algorithm as iOS app, no ARKit dependency.

---

## 1. System Requirements

### Minimum (DV-SLAM only)
```
Ubuntu 20.04+ / macOS 12+
CMake ≥ 3.16
GCC ≥ 9 / Clang ≥ 12 (C++17)
Eigen3
Python 3.8+
```

### Full Baselines (FAST-LIO, FAST-LIVO2, RTAB-Map)
```
ROS Noetic (Ubuntu 20.04) — required for ROS-based baselines
  sudo apt install ros-noetic-desktop-full
  source /opt/ros/noetic/setup.bash

Additional ROS packages:
  sudo apt install ros-noetic-pcl-ros \
                   ros-noetic-cv-bridge \
                   ros-noetic-image-transport \
                   ros-noetic-tf2-ros

RTAB-Map:
  sudo apt install ros-noetic-rtabmap-ros

Livox SDK (for FAST-LIO / FAST-LIVO2):
  git clone https://github.com/Livox-SDK/Livox-SDK2
  cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j4 && sudo make install
```

---

## 2. Quick Start

### Step 1: Install dependencies
```bash
# Core
sudo apt install libeigen3-dev cmake build-essential
pip install numpy scipy

# Baselines (optional)
pip install kiss-icp
```

### Step 2: Clone baselines
```bash
cd simulator/baselines
git clone --depth 1 https://github.com/hku-mars/FAST_LIO
git clone --depth 1 https://github.com/hku-mars/FAST-LIVO2
```

### Step 3: Build DV-SLAM simulator
```bash
cd simulator
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Step 4: Download ScanNet++ data
```bash
# Request access: https://kaldir.vc.in.tum.de/scannetpp/
# Download iPhone data for target scenes
python -m scannetpp.download raw --split Training --video_id <scene_id> \
  --raw_dataset_assets depth lowres_wide pose_intrinsic_imu mesh

# Extract depth frames
python -m iphone.prepare_iphone_data configs/prepare_iphone_data.yml
```

### Step 5: Run evaluation
```bash
# Single scene
./build/dv_sim /path/to/scannetpp/data/<scene_id> --preset full --output ./results/<scene_id>

# All presets
./scripts/batch_eval.sh /path/to/scannetpp/data ./results

# All baselines (DV-SLAM + ARKit + KISS-ICP + FAST-LIO + FAST-LIVO2)
./scripts/run_baselines.sh /path/to/scannetpp/data/<scene_id> ./results/<scene_id>

# Evaluate + generate LaTeX tables
python scripts/evaluate.py ./results
```

---

## 3. Presets (DV-SLAM Ablation)

| Preset | Description | IMU | LIO | Confidence | TLS | Stride |
|--------|-------------|-----|-----|------------|-----|--------|
| `full` | Full DV-SLAM | ✅ | ✅ | ✅ | ✅ | 4 |
| `noConf` | No confidence weight | ✅ | ✅ | ❌ | ✅ | 4 |
| `noTLS` | No TLS robust kernel | ✅ | ✅ | ✅ | ❌ | 4 |
| `sparse100` | ~100 pts/frame | ✅ | ✅ | ✅ | ✅ | 12 |
| `dense3000` | ~3000 pts/frame | ✅ | ✅ | ✅ | ✅ | 1 |
| `imuOnly` | Dead reckoning | ✅ | ❌ | — | — | 4 |

---

## 4. Baselines

| Baseline | Method | Script | Requirements |
|----------|--------|--------|-------------|
| **DV-SLAM** | Our LIO | `./build/dv_sim` | Eigen3 |
| **ARKit** | Apple VIO (from ScanNet++) | `scripts/baseline_arkit.py` | numpy, cv2 |
| **KISS-ICP** | Point cloud ICP | `scripts/baseline_kiss_icp.py` | `pip install kiss-icp` |
| **FAST-LIO** | LiDAR-Inertial | ROS launch | ROS Noetic + Livox SDK |
| **FAST-LIVO2** | LiDAR-Inertial-Visual | ROS launch | ROS Noetic + Livox SDK + OpenCV |
| **RTAB-Map** | RGB-D SLAM | `scripts/baseline_rtabmap.py` | `ros-noetic-rtabmap-ros` |

### FAST-LIO / FAST-LIVO2 (ROS required)
```bash
# 1. Build
cd baselines/FAST_LIO
mkdir -p catkin_ws/src && cd catkin_ws/src
ln -s ../../FAST_LIO .
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release

# 2. Convert ScanNet++ to ROS bag
python scripts/scannetpp_to_rosbag.py /path/to/scene --output /tmp/scene.bag

# 3. Run
source catkin_ws/devel/setup.bash
roslaunch fast_lio mapping_avia.launch &
rosbag play /tmp/scene.bag

# 4. Save trajectory
rostopic echo -b /tmp/scene.bag -p /Odometry > trajectory.csv
```

### RTAB-Map (ROS or standalone)
```bash
# ROS
roslaunch rtabmap_ros rgbd_mapping.launch \
  rgb_topic:=/camera/rgb depth_topic:=/camera/depth \
  camera_info_topic:=/camera/info

# Standalone CLI
rtabmap-rgbd_dataset --Rtabmap/DetectionRate 2 <rgb_dir> <depth_dir>
```

---

## 5. Output Structure

```
results/<scene_id>/
├── dvsim_full/
│   ├── estimated_trajectory.txt   (TUM: ts tx ty tz qx qy qz qw)
│   ├── gt_trajectory.txt
│   ├── reconstruction.ply
│   └── report.txt
├── dvsim_noConf/
├── dvsim_sparse100/
├── arkit/
│   ├── reconstruction.ply
│   └── arkit_trajectory.txt
├── kiss_icp/
│   ├── estimated_trajectory.txt
│   └── gt_trajectory.txt
└── evaluation_results.json
```

---

## 6. ScanNet++ Data Format

```
scene_id/
├── iphone/
│   ├── pose_intrinsic_imu.json    # Per-frame: pose(4×4), aligned_pose(4×4),
│   │                               #   intrinsic(3×3), timestamp, imu(15 floats)
│   ├── depth.bin                   # LZ4-compressed uint16 mm (raw)
│   ├── depth/                      # [after extraction] 16-bit PNG, 256×192, mm
│   ├── rgb.mkv                     # 60fps H.264 (raw)
│   └── rgb/                        # [after extraction] JPEG, 1920×1440
└── scans/
    └── mesh_aligned_0.05.ply       # FARO laser scan GT mesh
```

### IMU format (CMDeviceMotion, 15 floats per sample)
| Index | Field | Unit |
|-------|-------|------|
| 0-2 | rotationRate (x,y,z) | rad/s |
| 3-5 | userAcceleration (x,y,z) | g |
| 6-8 | magneticField (x,y,z) | μT |
| 9-11 | attitude (roll,pitch,yaw) | rad |
| 12-14 | gravity (x,y,z) | g |

**Conversion**: `specific_force = -(gravity + userAcceleration) * 9.81`

### Depth intrinsics
No LiDAR intrinsics provided. Scale RGB intrinsics:
```
fx_depth = fx_rgb × (256 / 1920)
fy_depth = fy_rgb × (192 / 1440)
```

---

## 7. Metrics

| Metric | Tool | Formula |
|--------|------|---------|
| ATE RMSE | `evaluate.py` / `evo` | √(Σ‖t_est - t_gt‖² / N) |
| RPE Trans | `evaluate.py` | Relative pose translation error |
| RPE Rot | `evaluate.py` | Relative pose rotation error (deg) |
| Map Accuracy | CloudCompare | Cloud-to-mesh distance (mean) |
| Map Completeness | CloudCompare | Mesh coverage at threshold |
| F-score | CloudCompare | Precision × Recall @ 5cm |

```bash
# Using evo (optional, more features)
pip install evo
evo_ape tum gt_trajectory.txt estimated_trajectory.txt --align --plot
```

---

## 8. Algorithm

Same C++ code as DV-SLAM iOS app:
- ESKF 18D state: R(SO3) + p(3) + v(3) + bg(3) + ba(3) + g(3)
- IMU midpoint integration (100Hz predict)
- Point-to-plane ICP with TLS robust kernel
- Bundle & Discard preprocessing (10cm voxel, density+confidence gate)
- Visual feature tracking (KLT optical flow)
- Voxel hash map (5cm, 500K cap, LRU eviction)

**No ARKit VIO pose used** — pure IMU + dToF LiDAR + Camera.
