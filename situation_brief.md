# DV-SLAM 논문 상황 브리핑

## 한 줄 요약
스마트폰 내장 dToF LiDAR(256×192, ~300-500pts/frame)와 IMU만으로 on-device LiDAR-Inertial Odometry를 구현했다. 이 sparse regime에서 LIO를 돌린 논문은 기존에 없으며, ScanNet++ (ICCV 2023) 공개 데이터셋으로 평가 가능하다. IEEE RA-L 타겟.

---

## 1. 내가 만든 것

iPhone 15 Pro Max의 **내장 센서만** 사용하여 on-device real-time(30Hz) LiDAR-Inertial Odometry(LIO)를 구현했다.

**입력 센서 3개** (전부 폰 내장):
- dToF depth image (256×192, 30Hz) — Apple SPAD 기반 dToF 센서
- IMU (100Hz, Core Motion CMDeviceMotion — gravity + userAcceleration → specific force, rotationRate → angular velocity)
- Monocular camera (1920×1440, 30Hz)

**파이프라인**:
IMU → ESKF predict → dToF depth를 camera intrinsics로 unproject해서 point cloud 생성 → point-to-plane ICP → visual feature tracking → 단일 ESKF update

**핵심**: ARKit의 VIO pose를 일절 사용하지 않음. raw sensor data에서 직접 pose를 추정한다.

**참고 — confidence map에 대한 결정:**
- dToF 센서는 per-pixel confidence (0/1/2)도 출력하며, 시스템에서 ICP weighting에 활용 중
- 그러나 이것은 센서 하드웨어가 뱉는 값을 가져다 쓴 것이지, 자체 추정/학습한 것이 아님 (`R_obs = σ²/w(c_i)` 한 줄)
- Contribution으로 내세우기엔 trivial → **핵심 기여에서 제외**, ablation의 한 항목으로 격하
- 이 결정으로 **confidence가 없는 ScanNet++ 데이터셋에서 평가 가능**해짐

---

## 2. 기존 연구 지형

### Category A: SLAM 알고리즘 논문 (기계식 LiDAR, RA-L/T-RO급)

| 논문 | venue | 센서 | 핵심 |
|------|-------|------|------|
| [FAST-LIO2](https://ieeexplore.ieee.org/document/9697912/) (Xu et al.) | RA-L 2022 | Livox, Velodyne | ikd-Tree + ESIKF, direct registration |
| [KISS-ICP](https://arxiv.org/abs/2209.15397) (Vizzo et al.) | RA-L 2023 | 다양한 기계식 LiDAR | point-to-point ICP, adaptive thresholding |
| [GenZ-ICP](https://arxiv.org/abs/2411.06766) (Lee et al.) | RA-L 2025 | 기계식 LiDAR | PCA 기반 adaptive weighting, degeneracy-robust |
| [FAST-LIVO2](https://ieeexplore.ieee.org/document/10757429/) (Zheng et al.) | T-RO 2024 | Livox + camera | LiDAR-Inertial-Visual, ESIKF |
| [D²-LIO](https://arxiv.org/html/2508.14355v1) (2025) | arxiv | 기계식 LiDAR | directional degeneracy-aware |
| [RTAB-Map](https://arxiv.org/abs/2403.06341) (Labbé et al.) | J. Field Robotics 2019 | RGB-D / LiDAR / Stereo | graph-based SLAM, loop closure |

**공통점**: 전부 10K-100K pts/frame, 로봇/드론/자동차 플랫폼

### Category B: 스마트폰 LiDAR 논문 (정확도 평가, SLAM 기여 없음)

| 논문 | venue | 내용 |
|------|-------|------|
| [iPad Pro LiDAR: 3D Rapid Mapping Tests](https://isprs-archives.copernicus.org/articles/XLIII-B1-2021/63/2021/) (Spreafico et al.) | ISPRS 2021 | iPad dToF 정확도 평가, 1:200 건축 도면용 |
| [iPhone 12 Pro LiDAR for Geosciences](https://www.nature.com/articles/s41598-021-01763-9) (Luetzenburg et al.) | Scientific Reports 2021 | ±1cm 소형 물체, ±10cm 대형 절벽 |
| [Apple LiDAR for Cultural Heritage](https://www.mdpi.com/2072-4292/14/17/4157) | Remote Sensing 2022 | 문화재 3D 스캔 정확도 |
| [Indoor Mapping Accuracy Comparison](https://www.tandfonline.com/doi/full/10.1080/16874048.2024.2408839) | J. Spatial Science 2024 | Apple dToF vs TLS 비교 |
| [Mobile Phone Based Indoor Mapping](https://isprs-archives.copernicus.org/articles/XLVIII-2-2024/415/2024/) (Strecha et al.) | ISPRS 2024 | iPhone LiDAR+카메라, drift 보정에 AutoTag 사용 |

**공통점**: 전부 ARKit 블랙박스에 의존, 자체 odometry 알고리즘 기여 없음

### Category C: 스마트폰 + 외장 LiDAR

| 논문 | venue | 내용 |
|------|-------|------|
| [Android + Livox Mid-360](https://isprs-annals.copernicus.org/articles/X-G-2025/375/2025/) (Huai et al.) | ISPRS 2025 | 외장 기계식 LiDAR를 폰에 장착, 폰 내장 dToF 아님 |

### RTAB-Map iOS
- App Store에 있는 공개 SLAM 앱이지만, odometry front-end를 **ARKit VIO에 완전 의존**
- RTAB-Map 자체 기여는 loop closure + graph optimization (back-end)
- 자체 LiDAR odometry 없음 ([GitHub issue #1026](https://github.com/introlab/rtabmap/issues/1026))
- RTAB-Map은 **Visual SLAM** — RGB+Depth만 있으면 되므로 TUM RGB-D, ScanNet++ 등 대부분의 데이터셋에서 평가 가능
- 내 시스템은 **LIO** — IMU+Depth가 필수이므로 사용 가능한 데이터셋이 제한적

### 빈 자리

|  | SLAM 알고리즘 기여 | 스마트폰 내장 dToF |
|--|:--:|:--:|
| Category A (RA-L급) | ✅ | ❌ |
| Category B (Sensors급) | ❌ | ✅ |
| Category C (ISPRS) | △ | ❌ (외장) |
| **내 논문** | **✅** | **✅** |

**스마트폰 내장 dToF로 자체 LIO를 구현한 논문이 없다.**

---

## 3. Contributions (confidence 제외 후 재구성)

### C1: Sparse dToF Regime에서 동작하는 최초의 On-Device LIO
- 기존 LIO (FAST-LIO2 등)는 10K-100K pts/frame을 전제
- 스마트폰 dToF는 ~300-500 pts/frame — **20-200배 더 sparse**
- 이 regime에서 LIO가 동작함을 최초로 실증
- ARKit VIO에 의존하지 않는 독립적 odometry

### C2: Sparse Regime Characterization
- Point-count sweep 실험으로 LIO collapse boundary 최초 식별
- "몇 개의 point가 있어야 LIO가 유지되는가"에 대한 체계적 분석
- 기존 논문에서 다루지 않은 regime

### C3: Lightweight Mobile LIO Architecture
- FAST-LIO2의 데이터 구조를 모바일 제약에 맞게 재설계:
  - ikd-tree → O(1) voxel hash map + LRU eviction (500K cap)
  - 10K-100K obs → max 100 obs stride sampling (LDLT 125x 절감)
  - PCL/ROS/Boost/Sophus 의존성 → Eigen-only self-contained
  - 고정 크기 배열 VoxelCell[20], KNN[5], ring buffer로 heap 할당 제거
  - Depth + visual residuals를 단일 ESKF update에 스택
- iPhone A17 Pro에서 30Hz real-time, 외장 하드웨어 불필요

> Confidence weighting (`R_obs = σ²/w(c_i)`)은 시스템에 포함되어 있으나, 센서가 제공하는 값을 그대로 사용한 것이므로 ablation table의 한 항목으로만 보고함.

---

## 4. 실험 평가 전략

### 4-1. ScanNet++ (ICCV 2023) — 주력 공개 데이터셋

[ScanNet++](https://github.com/scannetpp/scannetpp) — 460개 실내 씬, iPhone 13 Pro로 수집, FARO GT mesh 제공

**제공 데이터와 내 시스템 호환성:**

| 데이터 | ScanNet++ | 내 시스템 필요 | 호환? |
|--------|:---------:|:-------------:|:-----:|
| Depth map (256×192, 16bit mm) | ✅ | ✅ | ✅ |
| IMU (CMDeviceMotion) | ✅ | ✅ | **✅ 동일 포맷** |
| Camera intrinsics | ✅ | ✅ | ✅ |
| RGB | ✅ | ✅ (visual tracking) | ✅ |
| FARO GT mesh (sub-mm) | ✅ | 평가용 | ✅ |
| Confidence map | ❌ | 선택사항 (C1 제외됨) | **불필요** |

**핵심**: 내 앱의 SLAMService.mm도 `CMDeviceMotion`에서 `-(gravity + userAcceleration) * 9.81`로 specific force 변환 → ScanNet++의 IMU와 **동일한 소스, 동일한 변환**. 호환 문제 없음.

**ScanNet++으로 가능한 실험:**

| 실험 | 내용 | 가능? |
|------|------|:-----:|
| Trajectory 정확도 | LIO 출력 vs FARO-aligned pose | ✅ |
| Map quality | 재구성 point cloud vs FARO mesh (accuracy/completeness) | ✅ |
| Point-count sweep | stride 변경으로 point 수 조절 → collapse boundary 식별 | ✅ |
| Ablation | camera ON/OFF, ICP 변형, B&D ON/OFF 등 | ✅ |
| Runtime | per-frame latency breakdown | ✅ (자체 수집으로) |
| Confidence ablation | ON/OFF 비교 | ❌ (confidence 없음 → 자체 수집으로) |

**남은 확인사항**: depth frame과 IMU의 timestamp 동기화 품질 — 실제 데이터 받아서 테스트 필요

### 4-2. 자체 수집 데이터 — 보조

| 시퀀스 | 환경 | 목적 |
|--------|------|------|
| S1 | 일반 실내 | 기본 성능 + confidence ablation |
| S2 | 긴 복도 | 직선 drift |
| S3 | 빠른 회전 | IMU+ICP robustness |
| S4 | 유리/반사면 | 어려운 환경 |
| S5 | 야외 | 환경 다양성 |

- 전부 closed-loop (시작=끝) → drift 측정
- Confidence ablation은 자체 수집에서만 가능

### 4-3. GT 전략

| 방법 | 용도 | 정밀도 |
|------|------|--------|
| ScanNet++ FARO mesh | map quality, trajectory (aligned_poses) | sub-mm (mesh) |
| COLMAP pseudo-GT | 자체 수집 trajectory 평가 | cm급 |
| Closed-loop drift | 자체 수집 drift 평가 | 상대적 |

### 4-4. Baseline 비교

| Baseline | 설명 | 비교 의미 |
|----------|------|----------|
| Naive ICP (uniform weight, no IMU) | point cloud ICP만 | LIO tight coupling 효과 |
| IMU-only | ESKF predict만, ICP 없음 | Dead reckoning 하한선 |
| −Camera | dToF + IMU only | Visual tracking 기여 |
| KISS-ICP (sparse input) | 기존 LiDAR odometry | Sparse에서 기존 방법 실패 입증 |
| RTAB-Map iOS | ARKit VIO 의존 | "블랙박스 vs 독립 LIO" |

---

## 5. 센서 데이터 흐름 (참고)

```
iPhone Hardware
├── dToF SPAD sensor → ARKit API → sceneDepth.depthMap (256×192 float, per-pixel meters)
│                                 → sceneDepth.confidenceMap (256×192 uint8, 0/1/2)
├── Camera → ARKit API → capturedImage (1920×1440 YCbCr)
│                       → camera.intrinsics (fx, fy, cx, cy)
│                       → camera.transform (VIO pose — 미사용)
└── IMU → Core Motion API → CMDeviceMotion (100Hz)
                            ├── gravity (x,y,z)
                            ├── userAcceleration (x,y,z)
                            └── rotationRate (x,y,z)

내 앱의 변환:
  specific_force = -(gravity + userAcceleration) * 9.81
  angular_velocity = rotationRate
  → pushIMU(timestamp, specific_force, angular_velocity) → ESKF predict
```

- ARKit은 depth/confidence/camera를 제공하지만, **raw IMU는 제공 안 함** (별도 Core Motion)
- ARKit은 내부적으로 IMU를 VIO에 사용하지만 외부 노출 안 함
- dToF는 point cloud가 아닌 2D depth image 출력 → 앱이 intrinsics로 unproject → 3D point cloud
- ScanNet++의 수집 앱(ARKit-Scanner)도 동일한 CMDeviceMotion을 저장 → **포맷 호환**

---

## 6. 논의 포인트

1. **"sparse dToF에서 최초 LIO" — systems paper로 RA-L 충분한가?** Engineering contribution이지만 KISS-ICP도 비슷한 포지션으로 RA-L accept됨 (새 알고리즘 없이 "잘 동작하는 시스템")
2. **ScanNet++ 실험이 충분한가?** 460개 씬 중 몇 개로 평가하면 되는가?
3. **Baseline이 fair한가?** KISS-ICP를 sparse dToF에 돌리면 당연히 실패할텐데, 그게 의미있는 비교인가?
4. **논문 프레이밍**: "sparse dToF regime에서의 최초 LIO"로 포지셔닝하는 것이 적절한가?
5. **RTAB-Map iOS와의 비교**가 meaningful한 baseline이 될 수 있는가?

---

## 7. 기술 스펙 요약

| 항목 | 값 |
|------|-----|
| 디바이스 | iPhone 15 Pro Max (A17 Pro SoC) |
| dToF 해상도 | 256×192 (49,152 pixels) |
| 유효 point 수 | ~300-500 pts/frame |
| IMU | 100Hz (Core Motion CMDeviceMotion) |
| Camera | 1920×1440, 30Hz |
| ESKF state | 18D: rotation(3) + position(3) + velocity(3) + gyro bias(3) + accel bias(3) + gravity(3) |
| Map 구조 | Voxel hash map (10mm voxel, 500K cap, LRU eviction) |
| ICP method | Point-to-plane, TLS robust kernel |
| 실행 속도 | 30Hz on-device real-time |
| ARKit 의존 | depth+camera 수신만 (sensor HAL). VIO pose 미사용 |
| 빌드/테스트 | 완료, App Store 배포 중 |
| 평가 데이터셋 | ScanNet++ (ICCV 2023, 460 scenes, FARO GT) + 자체 수집 5 sequences |
