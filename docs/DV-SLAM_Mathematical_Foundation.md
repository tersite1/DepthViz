# DV-SLAM: Depth-Visual SLAM for Mobile LiDAR — Mathematical Foundation

**DV-SLAM: iPhone dToF LiDAR 기반 LiDAR-Inertial Odometry 시스템의 수학적 기초**

> 본 문서는 DV-SLAM 알고리즘의 수학적 정의, 유도, 구현 세부사항을 논문 투고 수준으로 기술한다.
> 모든 수식은 소스 코드와 1:1 대응하며, `📁 파일:라인` 형식으로 참조를 명시한다.
> 경로 기준: `DepthViz/Domain/Algorithm/DepthViz/` (SLAM 코어), `DepthViz/Domain/` (렌더러/셰이더), `DepthViz/Domain/Algorithm/Bridge/` (브릿지).

---

## Abstract

We present DV-SLAM (Depth-Visual SLAM), a LiDAR-Inertial Odometry (LIO) system designed for Apple iPhone's direct Time-of-Flight (dToF) LiDAR sensor (256$\times$192 pixels). Unlike existing LIO frameworks — FAST-LIO2 [1], DLIO [3], FAST-LIVO2 [2], Super-LIO [4] — which are built for mechanical spinning LiDARs producing dense point clouds ($>$100K points/scan), DV-SLAM addresses three fundamental challenges of mobile dToF sensing: (i) an order-of-magnitude sparser depth field ($\approx$49K pixels), (ii) per-pixel quality metadata via ARKit confidence scores (Low/Medium/High), and (iii) availability of an Apple ARKit Visual-Inertial Odometry (VIO) pose prior as a continuous safety net.

Our system employs an 18-dimensional Error-State Kalman Filter (ESKF) on the $SO(3) \times \mathbb{R}^{15}$ manifold, coupled with Point-to-Plane ICP using a voxel hash map for efficient nearest-neighbor search. Three novel modules augment the classical LIO pipeline:

1. **Multi-level ARKit confidence gating** — a three-stage cascade (point-level hard gate, voxel-level density/quality gate, ICP-level observation noise weighting) that exploits per-pixel confidence metadata unavailable in mechanical LiDARs.
2. **Adaptive Bundle & Discard preprocessing** — quality-aware point reduction (80–95% rejection rate) that preserves high-confidence regions while discarding unreliable measurements, replacing the uniform voxel grid downsampling used in existing LIO systems.
3. **Histogram-based Surface Thinning** — a statistical method for detecting and removing double-wall artifacts caused by drift, without requiring loop closure.

An **export-only architecture** decouples real-time rendering (delegated to ARKit's Metal GPU pipeline at 30fps) from SLAM computation running on a background thread, enabling iterative EKF updates (3 Gauss-Newton iterations) and maintenance of a 2M-point global map without frame drops. A 4-phase post-processing pipeline (SLAM map selection, Surface Thinning, voxel downsampling, statistical outlier removal) produces high-quality point clouds at export time.

We provide complete mathematical derivations for all 57 numbered equations with explicit source code correspondence, covering Lie group formulations ($SO(3)/SE(3)$ exponential/logarithmic maps), ESKF prediction and iterated update with Joseph-form covariance, observation Jacobian derivation, robust kernel design, spatial hashing, observability analysis, and convergence properties of the iterated EKF.

**Keywords:** LiDAR-Inertial Odometry, Error-State Kalman Filter, Point-to-Plane ICP, mobile 3D scanning, iPhone dToF LiDAR, ARKit

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Notation](#2-notation)
3. [Lie Group Formulation](#3-lie-group-formulation)
4. [Coordinate Systems and Transformations](#4-coordinate-systems-and-transformations)
5. [Sensor Models](#5-sensor-models)
6. [Error-State Kalman Filter](#6-error-state-kalman-filter)
7. [Point-to-Plane ICP and LIO Backend](#7-point-to-plane-icp-and-lio-backend)
8. [Bundle and Discard Preprocessing](#8-bundle-and-discard-preprocessing)
9. [Post-Processing Pipeline](#9-post-processing-pipeline)
10. [Hybrid ARKit-LIO Fusion](#10-hybrid-arkit-lio-fusion)
11. [Comparison with Apple ARKit](#11-comparison-with-apple-arkit)
12. [DV-SLAM Novelties](#12-dv-slam-novelties)
13. [Computational Complexity](#13-computational-complexity)

14. [Error Analysis and Approximation Bounds](#14-error-analysis-and-approximation-bounds)

**Appendix A.** [Algorithm Pseudocode](#appendix-a-algorithm-pseudocode)
**Appendix B.** [Parameter Table](#appendix-b-parameter-table)
**Appendix C.** [Source File Index](#appendix-c-source-file-index)
**Appendix D.** [Related Work](#appendix-d-related-work)

---

## 1. Introduction

DV-SLAM (Depth-Visual SLAM)은 Apple iPhone의 direct Time-of-Flight (dToF) LiDAR 센서(256×192)를 주 관측 소스로 사용하는 **LiDAR-Inertial Odometry (LIO)** 시스템이다. 로봇/자율주행 분야의 기계식 LiDAR(Velodyne 128채널, Livox Avia 등)를 전제로 설계된 기존 LIO 시스템(FAST-LIO2 [1], DLIO [2], Super-LIO [3])과 달리, DV-SLAM은 다음 세 가지 핵심 특성을 갖는 모바일 dToF 환경에 특화되었다:

1. **저해상도 깊이** — 256×192 (≈49K 픽셀), 기계식 LiDAR 대비 1-2 오더 적은 포인트
2. **ARKit confidence** — 각 깊이 픽셀에 Low(0)/Medium(1)/High(2) 신뢰도 부여, 기존 LiDAR에 없는 품질 메타데이터
3. **ARKit 포즈 prior** — Apple의 Visual-Inertial Odometry가 상시 제공하는 6-DoF 포즈를 안전망으로 활용

DV-SLAM의 아키텍처는 **export-only** 설계를 채택한다: 실시간 렌더링은 ARKit GPU 파이프라인이 담당하고(30fps 보장), DV-SLAM은 백그라운드 스레드에서 ESKF + Point-to-Plane ICP를 수행하여 글로벌 맵을 구축한다. 최종 내보내기 시 4단계 후처리 파이프라인을 적용하여 고품질 포인트클라우드를 생성한다.

---

## 2. Notation

| 기호 | 정의 | 소속 |
|------|------|------|
| $\mathbf{R} \in SO(3)$ | 회전 행렬 (3×3, $\mathbf{R}^T\mathbf{R} = \mathbf{I}$, $\det\mathbf{R} = 1$) | 리 군 |
| $\mathbf{p}, \mathbf{t} \in \mathbb{R}^3$ | 위치/이동 벡터 | |
| $\mathbf{v} \in \mathbb{R}^3$ | 속도 벡터 (월드 좌표계) | |
| $\mathbf{b}_g, \mathbf{b}_a \in \mathbb{R}^3$ | 자이로/가속도 바이어스 | ESKF 상태 |
| $\mathbf{g} \in \mathbb{R}^3$ | 중력 벡터 (월드 좌표계, 기본값 $[0,0,-9.81]^T$) | ESKF 상태 |
| $\mathbf{T} \in SE(3)$ | 강체 변환 (4×4 동차 행렬) | 리 군 |
| $\boldsymbol{\omega} \in \mathfrak{so}(3)$ | 축-각도 벡터 (각속도 × 시간 또는 회전 로그) | 리 대수 |
| $\boldsymbol{\xi} = [\boldsymbol{\rho}; \boldsymbol{\omega}] \in \mathfrak{se}(3)$ | 트위스트 벡터 | 리 대수 |
| $[\mathbf{v}]_\times$ | $\mathbf{v}$의 반대칭(skew-symmetric) 행렬 | hat 연산 |
| $\text{Exp}(\cdot)$ | 리 대수 → 리 군 지수 사상 | |
| $\text{Log}(\cdot)$ | 리 군 → 리 대수 로그 사상 | |
| $\mathbf{J}_l$ | 좌측 야코비안 (left Jacobian) | SE(3) |
| $\delta\mathbf{x} \in \mathbb{R}^{18}$ | 오차 상태 벡터 | ESKF |
| $\mathbf{P} \in \mathbb{R}^{18 \times 18}$ | 오차 상태 공분산 | ESKF |
| $\mathbf{F} \in \mathbb{R}^{18 \times 18}$ | 상태 전이 야코비안 | ESKF |
| $\mathbf{Q} \in \mathbb{R}^{18 \times 18}$ | 프로세스 노이즈 공분산 | ESKF |
| $\mathbf{H} \in \mathbb{R}^{N \times 18}$ | 관측 야코비안 | ICP |
| $\mathbf{K} \in \mathbb{R}^{18 \times N}$ | 칼만 이득 | ESKF |
| $\mathbf{n} \in \mathbb{R}^3$ | 국부 평면 법선 (단위 벡터) | ICP |
| $r_i \in \mathbb{R}$ | 점-평면 잔차 (부호 있는 거리) | ICP |
| $\Delta t$ | IMU 샘플링 간격 (≈0.01s at 100Hz) | |

---

## 3. Lie Group Formulation

### 3.1 SO(3): 3D 회전 매니폴드

#### Definition 3.1 (Exponential Map — Rodrigues Formula)

> 📁 `include/DV_Types.h:95–104` — `SO3::Exp()`

축-각도 벡터 $\boldsymbol{\omega} \in \mathbb{R}^3$로부터 회전 행렬 $\mathbf{R} \in SO(3)$로의 사상:

$$
\text{Exp}(\boldsymbol{\omega}) = \mathbf{I}_3 + \frac{\sin\theta}{\theta}[\boldsymbol{\omega}]_\times + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times^2
\tag{1}
$$

여기서 $\theta = \|\boldsymbol{\omega}\|$이고, hat 연산자 $[\cdot]_\times : \mathbb{R}^3 \to \mathfrak{so}(3)$는:

> 📁 `include/DV_Types.h:145–151` — `SO3::hat()`

$$
[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}
\tag{2}
$$

**구현 형태.** 코드는 단위 축 $\hat{\mathbf{a}} = \boldsymbol{\omega}/\theta$와 $\mathbf{K} = [\hat{\mathbf{a}}]_\times$를 사용하여 등가적으로 계산한다 (`DV_Types.h:100–102`):

$$
\text{Exp}(\boldsymbol{\omega}) = \mathbf{I}_3 + \sin\theta \cdot \mathbf{K} + (1 - \cos\theta) \cdot \mathbf{K}^2
$$

$\mathbf{K} = [\boldsymbol{\omega}]_\times / \theta$이므로 (1)과 수학적으로 동치이다.

**소각도 분기** ($\theta < 10^{-10}$, `DV_Types.h:97–98`): 1차 Taylor 전개 $\text{Exp}(\boldsymbol{\omega}) \approx \mathbf{I}_3 + [\boldsymbol{\omega}]_\times$를 적용하여 $0/0$ 수치 불안정을 방지한다.

#### Definition 3.2 (Logarithmic Map)

> 📁 `include/DV_Types.h:107–142` — `SO3::Log()`

회전 행렬 $\mathbf{R}$로부터 축-각도 벡터 $\boldsymbol{\omega}$를 추출한다. 세 가지 분기로 수치 안정성을 확보한다:

**경우 1 — 소각도** ($\theta < 10^{-10}$, `DV_Types.h:113–114`):

$$
\boldsymbol{\omega} \approx \frac{1}{2}\text{vee}(\mathbf{R} - \mathbf{R}^T) = \frac{1}{2}\begin{bmatrix} R_{21} - R_{12} \\ R_{02} - R_{20} \\ R_{10} - R_{01} \end{bmatrix}
\tag{3}
$$

**경우 2 — 일반** ($10^{-10} \leq \theta < \pi - 10^{-6}$, `DV_Types.h:139–141`):

$$
\boldsymbol{\omega} = \text{vee}\left(\frac{\theta}{2\sin\theta}(\mathbf{R} - \mathbf{R}^T)\right), \quad \theta = \arccos\left(\text{clamp}\left(\frac{\text{tr}(\mathbf{R}) - 1}{2},\; -1,\; 1\right)\right)
\tag{4}
$$

**경우 3 — 근-$\pi$** ($\theta \geq \pi - 10^{-6}$, `DV_Types.h:120–137`):

$\sin\theta \to 0$으로 (4)의 분모가 불안정해진다. $\theta = \pi$일 때 $\mathbf{R} = 2\hat{\mathbf{a}}\hat{\mathbf{a}}^T - \mathbf{I}$이므로 $\mathbf{S} = (\mathbf{R} + \mathbf{I})/2$는 rank-1 행렬이다. 코드는 $\mathbf{S}$의 세 열 중 **최대 노름 열**을 정규화하여 축 $\hat{\mathbf{a}}$를 추출하고 (`DV_Types.h:123–132`), $\text{vee}(\mathbf{R} - \mathbf{R}^T)$과의 내적 부호로 방향을 결정한다 (`DV_Types.h:134–135`).

> **Remark.** 이 접근법은 `SelfAdjointEigenSolver`를 사용하는 완전 고유값 분해 대신, 직접적인 열 선택으로 $O(1)$ 복잡도를 달성한다.

### 3.2 SE(3): 강체 변환

#### Definition 3.3 (SE(3) Exponential Map)

> 📁 `include/DV_Types.h:190–207` — `SE3::Exp()`

트위스트 $\boldsymbol{\xi} = [\boldsymbol{\rho};\; \boldsymbol{\omega}] \in \mathbb{R}^6$으로부터 동차 변환 행렬로의 사상:

$$
\text{Exp}(\boldsymbol{\xi}) = \begin{bmatrix} \text{Exp}(\boldsymbol{\omega}) & \mathbf{J}_l \boldsymbol{\rho} \\ \mathbf{0}^T & 1 \end{bmatrix}
\tag{5}
$$

**좌측 야코비안** $\mathbf{J}_l \in \mathbb{R}^{3\times 3}$ (`DV_Types.h:202–204`):

$$
\mathbf{J}_l = \mathbf{I}_3 + \frac{1 - \cos\theta}{\theta}\mathbf{K} + \frac{\theta - \sin\theta}{\theta}\mathbf{K}^2
\tag{6}
$$

여기서 $\mathbf{K} = [\hat{\mathbf{a}}]_\times$, $\hat{\mathbf{a}} = \boldsymbol{\omega}/\theta$이다.

> **주의.** (6)의 계수는 (1)의 계수와 형태가 다르다. $[\boldsymbol{\omega}]_\times$ 대신 단위축 $\mathbf{K}$를 사용하므로:
> - Exp: $\sin\theta / \theta$, $(1-\cos\theta)/\theta^2$ — **$\theta$ 또는 $\theta^2$로 나눔**
> - $\mathbf{J}_l$: $(1-\cos\theta)/\theta$, $(\theta - \sin\theta)/\theta$ — **$\theta$로만 나눔**
>
> 이는 $\mathbf{K}$가 이미 $1/\theta$를 흡수했기 때문이다.

#### Definition 3.4 (SE(3) Logarithmic Map)

> 📁 `include/DV_Types.h:209–236` — `SE3::Log()`

**역 좌측 야코비안** $\mathbf{J}_l^{-1}$ (`DV_Types.h:219–229`):

$$
\mathbf{J}_l^{-1} = \mathbf{I}_3 - \frac{1}{2}[\boldsymbol{\omega}]_\times + \beta [\boldsymbol{\omega}]_\times^2
\tag{7}
$$

여기서:

$$
\beta = \frac{1}{\theta^2} - \frac{1 + \cos\theta}{2\theta \sin\theta}
$$

$\theta \to \pi$에서 $\sin\theta \to 0$이므로 분모가 불안정해진다. 이 경우 $(1+\cos\theta)/(2\theta\sin\theta) \to 0$ (분자가 분모보다 빨리 0에 수렴)이므로 $\beta \to 1/\theta^2$으로 대체한다 (`DV_Types.h:222–225`).

---

## 4. Coordinate Systems and Transformations

### 4.1 좌표계 정의

| 좌표계 | 축 방향 | 출처 |
|--------|---------|------|
| 카메라 내부 | $x$→우, $y$→하, $z$→전방 (깊이) | 핀홀 카메라 모델 |
| ARKit 월드 | $x$→우, $y$→상(중력 반대), $z$→후방 | Apple ARKit |
| SLAM 월드 | ARKit 월드와 동일 (보정 전) | DV-SLAM ESKF |

### 4.2 flipYZ 변환

> 📁 `Renderer.swift:663–675` — `makeRotateToARCameraMatrix()`

카메라 내부 좌표계에서 ARKit 월드 좌표계로 변환하려면 Y축과 Z축을 모두 반전해야 한다:

$$
\mathbf{M}_{\text{flipYZ}} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & -1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
\tag{8}
$$

**이론적 근거.** ARKit의 $y$축은 중력 반대 방향(위)이고 $z$축은 카메라 뒤쪽이다. 카메라 내부 좌표계는 $y$축이 아래, $z$축이 전방이므로, 카메라→ARKit 변환 시 두 축 모두 부호 반전이 필요하다. 단순히 $z$만 반전하는 flipZ를 사용하면 $y$축 방향 불일치로 뷰 의존적 드리프트가 발생한다.

### 4.3 GPU 포인트 변환

> 📁 `Renderer.swift:345` — `pointCloudUniforms.localToWorld`

GPU 셰이더에서 각 깊이 픽셀을 월드 좌표로 변환하는 행렬:

$$
\mathbf{T}_{\text{localToWorld}} = \mathbf{V}^{-1} \cdot \mathbf{M}_{\text{flipYZ}} \cdot \mathbf{R}_z(\alpha)
\tag{9}
$$

여기서 $\mathbf{V}$는 ARKit 카메라 뷰 행렬, $\mathbf{R}_z(\alpha)$는 디바이스 방향(portrait: $\alpha = 90°$)에 따른 회전이다.

---

## 5. Sensor Models

### 5.1 LiDAR 깊이 역투영 (Pinhole Model)

iPhone dToF LiDAR는 256×192 해상도의 Float32 깊이맵을 30Hz로 제공한다. 각 깊이 픽셀 $(u, v)$를 카메라 좌표계 3D 점으로 역투영한다.

#### GPU 경로 (실시간 렌더링)

> 📁 `Metal/Shaders.metal:38–43` — `worldPoint()`

$$
\mathbf{p}_{\text{cam}} = \mathbf{K}^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} \cdot d
\tag{10}
$$

$\mathbf{K}^{-1}$은 역 내부 행렬이며 Swift에서 `cameraIntrinsicsInversed`로 전달된다. GPU는 이를 `localToWorld` 행렬로 즉시 월드 좌표로 변환한다.

#### CPU 경로 (SLAM 입력)

> 📁 `Bridge/SLAMService.mm:431–433`

$$
\mathbf{p}_{\text{cam}} = \begin{bmatrix} (u - c_x) \cdot d / f_x \\ (v - c_y) \cdot d / f_y \\ d \end{bmatrix}
\tag{11}
$$

**내부 파라미터 스케일링** (`SLAMService.mm:393–401`): ARKit의 카메라 내부 행렬은 캡처 이미지(1920×1440) 기준이므로 깊이맵 해상도(256×192)로 스케일링:

$$
f_x' = f_x \cdot \frac{W_{\text{depth}}}{W_{\text{cam}}}, \quad c_x' = c_x \cdot \frac{W_{\text{depth}}}{W_{\text{cam}}}
\tag{12}
$$

**서브샘플링** (`SLAMService.mm:403–404`): 4픽셀 간격으로 추출하여 프레임당 최대 $\lfloor 256/4 \rfloor \times \lfloor 192/4 \rfloor = 64 \times 48 = 3{,}072$개 점을 SLAM 엔진에 전달한다.

**유효성 필터** (`SLAMService.mm:422–428`):
- $d \leq 0$ 또는 $d > d_{\max}$ (사용자 설정 거리 제한) 또는 NaN → 기각
- ARKit confidence (0/1/2) < 사용자 임계값 × 2 → 기각

### 5.2 색상 변환 (YCbCr → RGB)

캡처 이미지는 NV12 (YCbCr 4:2:0 biplanar) 포맷이다. BT.601 변환:

> 📁 `Bridge/SLAMService.mm:449–454` (CPU)
> 📁 `Metal/Shaders.metal:30–33` (GPU)

$$
\begin{bmatrix} R \\ G \\ B \end{bmatrix} = \begin{bmatrix} 1 & 0 & 1.402 \\ 1 & -0.344 & -0.714 \\ 1 & 1.772 & 0 \end{bmatrix} \begin{bmatrix} Y \\ C_b - 128 \\ C_r - 128 \end{bmatrix}
\tag{13}
$$

결과는 $[0, 255]$로 클램핑 후 `DVPoint3D{x, y, z, r, g, b, confidence}` 구조체에 바인딩된다 (`DV_Types.h:42–72`).

### 5.3 IMU 모델

> 📁 `Renderer.swift:250–303` — `startIMUForSLAM()`
> 📁 `Bridge/SLAMService.mm:336–354` — `processIMUData:`

`CMMotionManager`에서 100Hz로 수집 (`Renderer.swift:262`).

**가속도** (`SLAMService.mm:341–345`):

$$
\mathbf{a}_{\text{raw}} = (\mathbf{a}_{\text{gravity}} + \mathbf{a}_{\text{user}}) \times 9.81 \;\text{m/s}^2
\tag{14}
$$

CoreMotion은 중력/사용자 가속도를 g 단위로 분리 제공하나, SLAM에는 합산 후 SI 단위로 변환하여 전달한다. 이는 ESKF가 중력 벡터 $\mathbf{g}$를 상태에 포함하여 직접 추정하기 때문이다.

**자이로스코프** (`SLAMService.mm:347–350`):

$$
\boldsymbol{\omega}_{\text{raw}} = [\omega_x, \omega_y, \omega_z]^T \;\text{rad/s}
\tag{15}
$$

`CMDeviceMotion.rotationRate`에서 스케일링 없이 직접 전달된다.

**측정 모델:**

$$
\boldsymbol{\omega}_{\text{meas}} = \boldsymbol{\omega}_{\text{true}} + \mathbf{b}_g + \mathbf{n}_g, \quad \mathbf{n}_g \sim \mathcal{N}(\mathbf{0},\, \sigma_g^2 \mathbf{I}_3)
\tag{16}
$$

$$
\mathbf{a}_{\text{meas}} = \mathbf{R}^T(\mathbf{a}_{\text{true}} - \mathbf{g}) + \mathbf{b}_a + \mathbf{n}_a, \quad \mathbf{n}_a \sim \mathcal{N}(\mathbf{0},\, \sigma_a^2 \mathbf{I}_3)
\tag{17}
$$

바이어스 동역학 (랜덤 워크):

$$
\dot{\mathbf{b}}_g = \mathbf{n}_{bg}, \quad \mathbf{n}_{bg} \sim \mathcal{N}(\mathbf{0},\, \sigma_{bg}^2 \mathbf{I}_3)
\tag{18}
$$

$$
\dot{\mathbf{b}}_a = \mathbf{n}_{ba}, \quad \mathbf{n}_{ba} \sim \mathcal{N}(\mathbf{0},\, \sigma_{ba}^2 \mathbf{I}_3)
\tag{19}
$$

**노이즈 파라미터** (`DV_Types.h:291–301`, `ESKFOptions`):

| 파라미터 | 기호 | 값 | 단위 |
|---------|------|-----|------|
| 자이로 노이즈 | $\sigma_g$ | 0.01 | rad/s/$\sqrt{\text{Hz}}$ |
| 가속도 노이즈 | $\sigma_a$ | 0.1 | m/s²/$\sqrt{\text{Hz}}$ |
| 자이로 바이어스 노이즈 | $\sigma_{bg}$ | 0.001 | rad/s²/$\sqrt{\text{Hz}}$ |
| 가속도 바이어스 노이즈 | $\sigma_{ba}$ | 0.01 | m/s³/$\sqrt{\text{Hz}}$ |

---

## 6. Error-State Kalman Filter

### 6.1 상태 정의

> 📁 `include/DV_Types.h:256–271` — `SysState` 구조체

DV-SLAM의 ESKF는 **18차원** 명목 상태(nominal state)를 유지한다:

$$
\mathbf{x} = \begin{bmatrix} \mathbf{R} \\ \mathbf{p} \\ \mathbf{v} \\ \mathbf{b}_g \\ \mathbf{b}_a \\ \mathbf{g} \end{bmatrix} \in SO(3) \times \mathbb{R}^{15}
\tag{20}
$$

| 상태 | 차원 | 초기값 | 소스 라인 |
|------|------|--------|----------|
| $\mathbf{R}$ | 3 (SO(3)) | $\mathbf{I}_{3\times3}$ | `DV_Types.h:258` |
| $\mathbf{p}$ | 3 | $\mathbf{0}$ | `DV_Types.h:259` |
| $\mathbf{v}$ | 3 | $\mathbf{0}$ | `DV_Types.h:260` |
| $\mathbf{b}_g$ | 3 | $\mathbf{0}$ | `DV_Types.h:261` |
| $\mathbf{b}_a$ | 3 | $\mathbf{0}$ | `DV_Types.h:262` |
| $\mathbf{g}$ | 3 | $[0, 0, -9.81]^T$ | `DV_Types.h:263` |

**오차 상태** $\delta\mathbf{x} \in \mathbb{R}^{18}$는 다음 순서로 정의된다 (`DV_ESKF.cpp:136`):

$$
\delta\mathbf{x} = \begin{bmatrix} \delta\boldsymbol{\theta} \\ \delta\mathbf{p} \\ \delta\mathbf{v} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{g} \end{bmatrix}, \quad \text{인덱스: [0{:}3,\; 3{:}6,\; 6{:}9,\; 9{:}12,\; 12{:}15,\; 15{:}18]}
\tag{21}
$$

**초기 공분산** (`DV_ESKF.cpp:9–11`):

$$
\mathbf{P}_0 = 10^{-4} \cdot \mathbf{I}_{18}, \quad \mathbf{P}_0[15{:}18,\; 15{:}18] = 1.0 \cdot \mathbf{I}_3
\tag{22}
$$

중력 블록에 더 큰 초기 불확실성($1.0$ vs $10^{-4}$)을 부여하여 초기 중력 방향 추정을 허용한다.

> **설계 결정 — 중력 $\mathbb{R}^3$ 표현.** FAST-LIO2 [1]는 중력을 $S^2$ 매니폴드(2 자유도, 크기 고정)로 표현하여 추정 자유도를 줄인다. DV-SLAM은 $\mathbb{R}^3$ (3 자유도)을 사용하되 매우 작은 프로세스 노이즈($10^{-10}$, §6.3)로 크기를 사실상 고정한다. 이 접근법은 구현이 단순하고, $S^2$ 매니폴드의 차트 전환 로직이 불필요하다는 장점이 있다.

### 6.2 예측 단계 (IMU Propagation)

> 📁 `src/DV_ESKF.cpp:19–70` — `predict()`

각 IMU 샘플마다 **중점 적분(midpoint integration)**으로 명목 상태를 전파한다.

**Step 1. 바이어스 보정** (`DV_ESKF.cpp:26–27`):

$$
\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_{\text{meas}} - \mathbf{b}_g, \quad \hat{\mathbf{a}} = \mathbf{a}_{\text{meas}} - \mathbf{b}_a
\tag{23}
$$

**Step 2. 회전 적분** (`DV_ESKF.cpp:30–33`):

$$
\Delta\mathbf{R} = \text{Exp}(\hat{\boldsymbol{\omega}} \cdot \Delta t)
\tag{24}
$$

$$
\mathbf{R}_{k+1} = \mathbf{R}_k \cdot \Delta\mathbf{R} \quad \text{(우측 곱 관례)}
\tag{25}
$$

**Step 3. 중점 가속도** (`DV_ESKF.cpp:36–37`):

$$
\mathbf{R}_{\text{mid}} = \mathbf{R}_k \cdot \text{Exp}(\hat{\boldsymbol{\omega}} \cdot \Delta t / 2)
\tag{26}
$$

$$
\mathbf{a}_{\text{world}} = \mathbf{R}_{\text{mid}} \cdot \hat{\mathbf{a}} + \mathbf{g}
\tag{27}
$$

중점 회전 $\mathbf{R}_{\text{mid}}$를 사용하여 가속도 회전 변환의 정확도를 높인다. $\mathbf{R}_k$나 $\mathbf{R}_{k+1}$ 대신 중간값을 사용하는 것은 2차 정확도를 제공한다.

**Step 4. 위치 및 속도** (`DV_ESKF.cpp:40–41`):

$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \cdot \Delta t + \frac{1}{2}\mathbf{a}_{\text{world}} \cdot \Delta t^2
\tag{28}
$$

$$
\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_{\text{world}} \cdot \Delta t
\tag{29}
$$

### 6.3 프로세스 노이즈

> 📁 `src/DV_ESKF.cpp:153–176` — `buildProcessNoise()`

$$
\mathbf{Q} = \text{diag}\left(\mathbf{Q}_\theta,\; \mathbf{Q}_p,\; \mathbf{Q}_v,\; \mathbf{Q}_{bg},\; \mathbf{Q}_{ba},\; \mathbf{Q}_g\right) \in \mathbb{R}^{18 \times 18}
\tag{30}
$$

| 블록 | 수식 | 라인 |
|------|------|------|
| $\mathbf{Q}_\theta$ | $\sigma_g^2 \Delta t \cdot \mathbf{I}_3$ | 163 |
| $\mathbf{Q}_p$ | $\sigma_a^2 \Delta t^3 / 4 \cdot \mathbf{I}_3$ | 165 |
| $\mathbf{Q}_v$ | $\sigma_a^2 \Delta t \cdot \mathbf{I}_3$ | 167 |
| $\mathbf{Q}_{bg}$ | $\sigma_{bg}^2 \Delta t \cdot \mathbf{I}_3$ | 169 |
| $\mathbf{Q}_{ba}$ | $\sigma_{ba}^2 \Delta t \cdot \mathbf{I}_3$ | 171 |
| $\mathbf{Q}_g$ | $10^{-10} \cdot \mathbf{I}_3$ (상수, $\Delta t$ 미곱) | 173 |

> **Remark (위치 노이즈 근사).** 연속시간 가속도 노이즈의 엄밀한 이산화는 위치에 $\sigma_a^2 \Delta t^5/20$의 분산을 유도한다 [4]. DV-SLAM은 간략화된 $\sigma_a^2 \Delta t^3/4$를 사용하는데, 이는 (a) 속도 노이즈 $\sigma_a^2 \Delta t$와 위치-속도 결합을 1차 근사한 것이며 (b) $\Delta t = 0.01\text{s}$에서 $\Delta t^5/20 \approx 5 \times 10^{-12}$, $\Delta t^3/4 \approx 2.5 \times 10^{-7}$로 후자가 보수적(더 큰)이어서 필터 안정성에 유리하다.

> **Remark (중력 노이즈).** $\mathbf{Q}_g = 10^{-10} \cdot \mathbf{I}_3$은 $\Delta t$에 곱하지 않는다 (`DV_ESKF.cpp:173`). 이는 중력이 물리적으로 시불변이므로 프로세스 노이즈를 사실상 0으로 만들되, 공분산 행렬의 양정치성을 유지하기 위해 미세한 값을 부여하는 것이다.

### 6.4 상태 전이 야코비안

> 📁 `src/DV_ESKF.cpp:43–67`

$18 \times 18$ 상태 전이 행렬 $\mathbf{F}$는 오차 상태 전파를 기술한다:

$$
\mathbf{F} = \begin{bmatrix}
\Delta\mathbf{R}^T & \mathbf{0} & \mathbf{0} & -\mathbf{I}_3\Delta t & \mathbf{0} & \mathbf{0} \\[4pt]
-\mathbf{R}_k[\hat{\mathbf{a}}]_\times \frac{\Delta t^2}{2} & \mathbf{I}_3 & \mathbf{I}_3\Delta t & \mathbf{0} & -\mathbf{R}_k\frac{\Delta t^2}{2} & \mathbf{I}_3\frac{\Delta t^2}{2} \\[4pt]
-\mathbf{R}_k[\hat{\mathbf{a}}]_\times \Delta t & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} & -\mathbf{R}_k\Delta t & \mathbf{I}_3\Delta t \\[4pt]
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} & \mathbf{0} \\[4pt]
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\[4pt]
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}
\tag{31}
$$

각 블록의 코드 대응:

| 블록 | 수식 | 라인 |
|------|------|------|
| $\mathbf{F}_{00}$ | $\Delta\mathbf{R}^T$ (`dR.R.transpose()`) | 47 |
| $\mathbf{F}_{09}$ | $-\mathbf{I}_3 \Delta t$ | 48 |
| $\mathbf{F}_{30}$ | $-\mathbf{R}_k [\hat{\mathbf{a}}]_\times \Delta t^2/2$ | 52 |
| $\mathbf{F}_{36}$ | $\mathbf{I}_3 \Delta t$ | 53 |
| $\mathbf{F}_{3,12}$ | $-\mathbf{R}_k \Delta t^2/2$ | 54 |
| $\mathbf{F}_{3,15}$ | $\mathbf{I}_3 \Delta t^2/2$ | 55 |
| $\mathbf{F}_{60}$ | $-\mathbf{R}_k [\hat{\mathbf{a}}]_\times \Delta t$ | 58 |
| $\mathbf{F}_{6,12}$ | $-\mathbf{R}_k \Delta t$ | 59 |
| $\mathbf{F}_{6,15}$ | $\mathbf{I}_3 \Delta t$ | 60 |

> **Remark (선형화 근사).** $\mathbf{F}$의 위치/속도 블록에서 $\mathbf{R}_k$ (업데이트 전 회전)를 사용한다 (`DV_ESKF.cpp:52,54,58,59`). 실제 적분은 $\mathbf{R}_{\text{mid}}$ (26)을 사용하므로 $\mathbf{F}$는 1차 선형화 근사이다. $\Delta t = 0.01\text{s}$에서 $\|\mathbf{R}_k - \mathbf{R}_{\text{mid}}\| = O(\|\hat{\boldsymbol{\omega}}\| \Delta t / 2) \leq O(10^{-3})$이므로 이 차이는 무시할 수 있다.

**공분산 전파** (`DV_ESKF.cpp:63–67`):

$$
\mathbf{P}_{k+1|k} = \mathbf{F} \cdot \mathbf{P}_{k|k} \cdot \mathbf{F}^T + \mathbf{Q}
\tag{32}
$$

대칭성 강제: $\mathbf{P} \leftarrow \frac{1}{2}(\mathbf{P} + \mathbf{P}^T)$ (`DV_ESKF.cpp:67`). 부동소수점 오차 누적에 의한 비대칭을 매 스텝 보정한다.

### 6.5 관측 업데이트 (Iterated EKF)

> 📁 `src/DV_ESKF.cpp:72–133` — `updateObserve()`

LiDAR 관측이 도착하면 **반복적 업데이트(IEKF)**를 수행한다:

**최대 반복 횟수:** 3회 (`DV_Types.h:292`)

**For** $\text{iter} = 0, 1, 2$:

**Step 1.** 관측 함수 호출 → $\mathbf{H} \in \mathbb{R}^{N \times 18}$, $\mathbf{r} \in \mathbb{R}^N$, $\mathbf{R}_{\text{obs}} \in \mathbb{R}^{N \times N}$ (§7 참조)

**Step 2. 혁신 공분산** (`DV_ESKF.cpp:103–104`):

$$
\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}_{\text{obs}}
\tag{33}
$$

**Step 3. 칼만 이득** (`DV_ESKF.cpp:105–111`):

$$
\mathbf{K} = \mathbf{P}\mathbf{H}^T \cdot \mathbf{S}^{-1}
\tag{34}
$$

$\mathbf{S}^{-1}$는 **LDLT 분해**로 계산한다 (`Eigen::LDLT`). 직접 역행렬 대신 LDLT를 사용하는 이유:
- $\mathbf{S}$는 대칭 양정치이므로 LDLT가 적합
- Cholesky보다 수치 안정 (거의 양반정치인 경우에도 작동)
- LDLT 실패 시 업데이트를 기각하고 상태를 백업에서 복원 (`DV_ESKF.cpp:106–110`)

구체적으로: $\mathbf{K} = \mathbf{P}\mathbf{H}^T \cdot \text{LDLT}(\mathbf{S})^{-1} \cdot \mathbf{I}_N$ (`DV_ESKF.cpp:111`)

**Step 4. 오차 상태 보정** (`DV_ESKF.cpp:114`):

$$
\delta\mathbf{x} = \mathbf{K} \cdot \mathbf{r}
\tag{35}
$$

**Step 5. 명목 상태 주입** (`DV_ESKF.cpp:135–151`, `applyCorrection()`):

$$
\mathbf{R} \leftarrow \mathbf{R} \cdot \text{Exp}(\delta\boldsymbol{\theta}) \quad \text{(우측 곱, line 145)}
\tag{36}
$$

$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}, \quad \mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v}
\tag{37}
$$

$$
\mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g, \quad \mathbf{b}_a \leftarrow \mathbf{b}_a + \delta\mathbf{b}_a, \quad \mathbf{g} \leftarrow \mathbf{g} + \delta\mathbf{g}
\tag{38}
$$

**Step 6. 공분산 업데이트 — Joseph 형태** (`DV_ESKF.cpp:121–123`):

$$
\mathbf{P} \leftarrow (\mathbf{I}_{18} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}_{18} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}_{\text{obs}}\mathbf{K}^T
\tag{39}
$$

> **Remark (Joseph 형태).** 표준 $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$보다 수치 안정적이다. 부동소수점 오차로 $\mathbf{K}\mathbf{H}$가 부정확해도 $\mathbf{P}$의 양정치성이 보장된다. 대칭성도 별도 강제한다 (`DV_ESKF.cpp:123`).

**Step 7. 수렴 판정** (`DV_ESKF.cpp:126`):

$$
\|\delta\mathbf{x}\|_2 < \epsilon_{\text{quit}} = 10^{-6} \implies \text{조기 종료}
\tag{40}
$$

**안전장치**: 관측 함수 실패 또는 LDLT 분해 실패 시, 반복 루프 전에 백업한 상태 $(\mathbf{x}_{\text{backup}}, \mathbf{P}_{\text{backup}})$으로 완전 복원한다 (`DV_ESKF.cpp:76–78, 88–91, 106–110`).

**수렴 성질.** IEKF (반복 업데이트)는 Mahalanobis 가중 잔차의 국소 최솟값으로 수렴한다:

**Proposition 6.1 (IEKF 축소 사상).** $\delta\mathbf{x}^{(k)}$를 $k$번째 반복의 보정 벡터라 하자. 관측 함수 $h(\mathbf{x})$ (Eq. 45)가 2회 연속 미분 가능하고, 야코비안 $\mathbf{H}(\mathbf{x})$가 $\mathbf{x}^*$ (진상태) 근방에서 Lipschitz 연속 (상수 $L_H$)이면:

$$
\|\delta\mathbf{x}^{(k+1)}\| \leq \gamma \|\delta\mathbf{x}^{(k)}\|, \quad \gamma = O(L_H \cdot \|\delta\mathbf{x}^{(0)}\|) < 1
$$

초기 오차가 충분히 작을 때 (즉, $\|\delta\mathbf{x}^{(0)}\| < 1/L_H$), 각 반복에서 축소율 $\gamma < 1$이 보장된다.

*근거.* DV-SLAM의 관측 함수 (Eq. 45–46)에서:
- 잔차 $r = \mathbf{n}^T(\mathbf{R}\mathbf{p} + \mathbf{t} - \bar{\mathbf{q}})$는 $\mathbf{t}$에 선형, $\mathbf{R}$에 해석적
- 회전 야코비안 $-\mathbf{n}^T\mathbf{R}[\mathbf{p}]_\times$는 $\delta\boldsymbol{\theta}$에 대해 매끄러움
- TLS 하드 절단 (Eq. 48)이 잔차를 $|r| \leq 0.10$m 범위로 제한하여 대편향 관측의 영향을 원천 차단

실무적으로 DV-SLAM에서 $\|\delta\mathbf{x}^{(1)}\| / \|\delta\mathbf{x}^{(0)}\| < 0.1$ (cm 스케일 초기 오차, 10cm TLS 경계)이므로, 3회 반복 (`DV_Types.h:292`)은 수렴에 충분하다. 실제로 대부분의 프레임에서 1–2회 반복 내에 $\|\delta\mathbf{x}\| < \epsilon_{\text{quit}} = 10^{-6}$에 도달한다.

### 6.6 관측가능성 분석 (Observability Analysis)

18차원 ESKF 상태의 관측가능성은 추정 수렴성과 정확도를 이해하는 데 핵심적이다.

**Definition 6.1 (국소 관측가능성).** 상태 $\mathbf{x} \in SO(3) \times \mathbb{R}^{15}$와 관측 모델 $h(\mathbf{x}) = \{r_i\}$ (점-평면 잔차)를 갖는 비선형 시스템이 $\mathbf{x}_0$ 근방에서 **국소 관측가능(locally observable)**이려면, 관측가능성 행렬:

$$
\mathcal{O} = \begin{bmatrix} \mathbf{H} \\ \mathbf{H}\mathbf{F} \\ \mathbf{H}\mathbf{F}^2 \\ \vdots \\ \mathbf{H}\mathbf{F}^{n-1} \end{bmatrix} \in \mathbb{R}^{nN \times 18}
\tag{40a}
$$

의 랭크가 $\mathbf{x}_0$ 근방에서 18이어야 한다.

**Proposition 6.2 (관측 가능 부분공간).** 일반적 운동(퇴화하지 않는 회전 및 이동) 하에서, 18차원 상태의 관측가능성은 다음과 같이 분석된다:

| 상태 성분 | 관측가능성 | 메커니즘 |
|-----------|-----------|---------|
| $\mathbf{R}$ (회전) | **직접** | LiDAR 점-평면 잔차 (Eq. 46): $\partial r/\partial \delta\boldsymbol{\theta} = -\mathbf{n}^T\mathbf{R}[\mathbf{p}]_\times \neq \mathbf{0}$ |
| $\mathbf{p}$ (위치) | **직접** | LiDAR 점-평면 잔차 (Eq. 46): $\partial r/\partial \delta\mathbf{p} = \mathbf{n}^T$ |
| $\mathbf{v}$ (속도) | **간접** | IMU 전파가 $\delta\mathbf{v}$를 $\delta\mathbf{p}$에 결합: $\mathbf{F}_{36} = \mathbf{I}_3\Delta t$ (Eq. 31) |
| $\mathbf{b}_g$ (자이로 바이어스) | **간접** | 회전 오차 누적: $\mathbf{F}_{09} = -\mathbf{I}_3\Delta t$가 $\delta\mathbf{b}_g$를 $\delta\boldsymbol{\theta}$에 결합 |
| $\mathbf{b}_a$ (가속도 바이어스) | **간접** | 위치 오차 결합: $\mathbf{F}_{3,12} = -\mathbf{R}\Delta t^2/2$가 $\delta\mathbf{b}_a$를 $\delta\mathbf{p}$에 결합 |
| $\mathbf{g}$ (중력) | **간접** | 위치/속도 결합: $\mathbf{F}_{3,15} = \mathbf{I}_3\Delta t^2/2$, $\mathbf{F}_{6,15} = \mathbf{I}_3\Delta t$ |

*증명 스케치.* 상태 전이 행렬 (Eq. 31)로부터:

1. $\mathbf{H}\mathbf{F}$는 속도 블록에 $\mathbf{n}^T \cdot \mathbf{I}_3\Delta t$를 포함하여, 연속 프레임의 위치 관측을 통해 $\mathbf{v}$를 관측가능하게 만든다.
2. $\mathbf{H}\mathbf{F}^2$는 $\mathbf{b}_a$ 블록에 $-\mathbf{n}^T\mathbf{R}\Delta t$를 도입하여, 가속도 바이어스를 관측가능하게 만든다.
3. 자이로 바이어스 $\mathbf{b}_g$는 회전-위치 결합을 통해 관측가능해지며, 이를 위해 디바이스가 회전 ($[\hat{\mathbf{a}}]_\times \neq \mathbf{0}$)해야 한다.
4. 중력 $\mathbf{g}$는 위치/속도에 대한 직접 결합 ($\mathbf{F}_{3,15}, \mathbf{F}_{6,15}$)을 통해 관측가능하다. $\square$

**퇴화 조건 (Degenerate Cases):**

| 조건 | 영향 | DV-SLAM 대응 |
|------|------|-------------|
| **평면 환경** | 모든 법선 $\mathbf{n}$이 평행 → $\text{rank}(\mathbf{H})$ 감소, 법선 방향의 위치만 관측 가능 | 평면성 검증 (Eq. 44)이 퇴화 평면을 기각 |
| **정지 상태** | $\boldsymbol{\omega} = \mathbf{0}$ → $\mathbf{b}_g$ 비관측가능 (자이로 바이어스와 영 회전 구분 불가) | ESKF 공분산이 불확실성 증가를 반영, ARKit fallback 발동 |
| **등속 운동** | $\mathbf{a} = \mathbf{0}$ → $\mathbf{b}_a$ 약 관측가능 (중력 결합만으로 간접 추정) | 보수적 프로세스 노이즈 (§6.3)가 약 관측가능 상태에서 필터 안정성 확보 |
| **좁은 FOV** | iPhone LiDAR FOV ≈ 60° → 다양한 법선 방향 부족 | Bundle & Discard가 품질 높은 점만 보존, 복셀 다양성 확보 |

> **Remark (DV-SLAM의 실용적 관측가능성 보장).** DV-SLAM의 Divergence Guard (§7.9)는 퇴화 시나리오의 영향을 완화한다: ESKF가 발산하면 (퇴화 환경에서 발생 가능성 높음) ARKit 포즈 리셋이 유계 오차 폴백을 제공한다. 이는 독립형 LIO 시스템 대비 실용적 장점이다. 또한, iPhone의 핸드헬드 특성상 사용자가 자연스럽게 회전/이동하므로, 실제 사용 환경에서 퇴화 조건이 지속되는 경우는 드물다.

---

## 7. Point-to-Plane ICP and LIO Backend

### 7.1 개요

> 📁 `src/DV_LIOBackend.cpp:37–194` — `process()`

LIO 백엔드는 각 키프레임에서 (a) IMU 전파된 ESKF 상태를 사전값으로 사용하고, (b) 현재 포인트클라우드와 글로벌 맵 간 Point-to-Plane ICP를 수행하여 포즈를 정제한다.

### 7.2 월드 좌표 변환

> 📁 `src/DV_LIOBackend.cpp:111–112`

$$
\mathbf{p}_{\text{world}} = \mathbf{R} \cdot \mathbf{p}_{\text{cam}} + \mathbf{t}
\tag{41}
$$

> **가정 ($\mathbf{T}_{\text{cam-imu}} \approx \mathbf{I}$).** 코드는 카메라-IMU 간 외부 파라미터를 단위 행렬로 가정한다 (`DV_LIOBackend.cpp:107–110`). iPhone에서 LiDAR와 IMU의 물리적 거리는 약 5–10mm이다. 이 레버암(lever-arm) 오차는 회전 시 최대 $\|\boldsymbol{\omega}\| \times 10\text{mm}$의 위치 편향을 유발하며, 일반적 핸드헬드 동작($\|\boldsymbol{\omega}\| \leq 1\text{rad/s}$)에서 ≤1cm이다. cm 수준 정밀도를 주장하는 논문에서는 이 가정을 명시해야 한다.

### 7.3 국부 평면 추정

> 📁 `include/DV_VoxelHashMap.h:164–197` — `fitPlane()`

#### 7.3.1 최근접 이웃 탐색 (Approximate KNN)

> 📁 `include/DV_VoxelHashMap.h:135–160` — `getTopK()`

질의점 $\mathbf{q}$가 속한 복셀과 26개 인접 복셀($3^3 = 27$)에서 $K = 5$개 최근접점을 검색한다. 해시 기반이므로 $O(1)$ 복셀 접근이며, 복셀당 최대 20개 점을 검사하여 최대 $27 \times 20 = 540$회 거리 계산을 수행한다. KNN 결과는 거리 내림차순으로 유지되며, 새 후보가 현재 최악보다 가까우면 교체한다 (`DV_VoxelHashMap.h:30–45`).

#### 7.3.2 공분산 행렬 고유값 분해

$K$개 이웃점 $\{\mathbf{q}_i\}_{i=1}^{K}$ (단, $K \geq 3$ 필요, `DV_VoxelHashMap.h:165`):

**중심점:**

$$
\bar{\mathbf{q}} = \frac{1}{K}\sum_{i=1}^{K}\mathbf{q}_i
\tag{42}
$$

(`DV_VoxelHashMap.h:168–172`)

**표본 공분산 행렬:**

$$
\mathbf{C} = \frac{1}{K}\sum_{i=1}^{K}(\mathbf{q}_i - \bar{\mathbf{q}})(\mathbf{q}_i - \bar{\mathbf{q}})^T \in \mathbb{R}^{3\times 3}_{\text{sym}}
\tag{43}
$$

(`DV_VoxelHashMap.h:175–180`)

**고유값 분해** (`DV_VoxelHashMap.h:183–187`):

$$
\mathbf{C} = \mathbf{U}\boldsymbol{\Lambda}\mathbf{U}^T, \quad \boldsymbol{\Lambda} = \text{diag}(\lambda_0, \lambda_1, \lambda_2), \quad \lambda_0 \leq \lambda_1 \leq \lambda_2
$$

`Eigen::SelfAdjointEigenSolver`를 사용한다. $\mathbf{C}$가 대칭 양반정치이므로 일반 SVD 대신 특화된 대칭 고유값 분해기를 사용하여 계산 효율을 높인다.

**법선 벡터:** $\mathbf{n} = \mathbf{u}_0$ (최소 고유값 $\lambda_0$에 대응하는 고유벡터, `eigenvectors().col(0)`)

**평면성 검증** (`DV_VoxelHashMap.h:189–194`):

$$
\text{reject if}\; \lambda_1 \leq 10^{-6} \;\text{(퇴화)} \quad \text{OR} \quad \frac{\lambda_0}{\lambda_1} > 0.3 \;\text{(비평면)}
\tag{44}
$$

$\lambda_0/\lambda_1 > 0.3$은 최소 분산 방향의 산포가 두 번째 주성분 대비 30% 이상임을 의미하며, 이는 점들이 평면보다는 선분이나 덩어리에 가까운 분포를 가짐을 나타낸다.

### 7.4 점-평면 잔차

> 📁 `src/DV_LIOBackend.cpp:123–125`

$$
r_i = \mathbf{n}^T(\mathbf{p}_{\text{world},i} - \bar{\mathbf{q}})
\tag{45}
$$

이는 점 $\mathbf{p}_{\text{world},i}$에서 법선 $\mathbf{n}$과 중심점 $\bar{\mathbf{q}}$로 정의되는 평면까지의 **부호 있는 거리(signed distance)**이다.

### 7.5 관측 야코비안

> 📁 `src/DV_LIOBackend.cpp:148–175`

ESKF 업데이트를 위한 관측 야코비안 $\mathbf{H}_i \in \mathbb{R}^{1 \times 18}$:

$$
\mathbf{H}_i = \begin{bmatrix}
\underbrace{-\mathbf{n}^T \mathbf{R} [\mathbf{p}_{\text{cam}}]_\times}_{\partial r / \partial \delta\boldsymbol{\theta}} &
\underbrace{\mathbf{n}^T}_{\partial r / \partial \delta\mathbf{p}} &
\underbrace{\mathbf{0}_{1\times 3}}_{\partial r / \partial \delta\mathbf{v}} &
\underbrace{\mathbf{0}_{1\times 3}}_{\partial r / \partial \delta\mathbf{b}_g} &
\underbrace{\mathbf{0}_{1\times 3}}_{\partial r / \partial \delta\mathbf{b}_a} &
\underbrace{\mathbf{0}_{1\times 3}}_{\partial r / \partial \delta\mathbf{g}}
\end{bmatrix}
\tag{46}
$$

**유도.** $r = \mathbf{n}^T(\mathbf{R}\mathbf{p}_{\text{cam}} + \mathbf{t} - \bar{\mathbf{q}})$에서:

- 회전 섭동 $\mathbf{R} \to \mathbf{R}\text{Exp}(\delta\boldsymbol{\theta})$:
  $\frac{\partial r}{\partial \delta\boldsymbol{\theta}} = \mathbf{n}^T \mathbf{R} \frac{\partial}{\partial \delta\boldsymbol{\theta}}[\text{Exp}(\delta\boldsymbol{\theta})\mathbf{p}_{\text{cam}}] \approx \mathbf{n}^T \mathbf{R} (-[\mathbf{p}_{\text{cam}}]_\times)$

  (`DV_LIOBackend.cpp:164–165`)

- 이동 섭동 $\mathbf{t} \to \mathbf{t} + \delta\mathbf{p}$: $\frac{\partial r}{\partial \delta\mathbf{p}} = \mathbf{n}^T$ (`DV_LIOBackend.cpp:168`)

LiDAR 관측은 회전과 위치만 직접 관측 가능하며, 속도/바이어스/중력은 IMU 예측 모델 (31)을 통해 간접 추정된다.

**잔차 부호 반전:** `residual(i) = -r_i` (`DV_LIOBackend.cpp:170`). ESKF 관례상 잔차를 $\mathbf{z} - h(\mathbf{x})$ 형태로 전달하므로 부호를 반전한다.

### 7.6 강건 가중치

> 📁 `include/DV_RobustKernels.h`

DV-SLAM은 두 가지 독립적 강건 커널을 적용한다. 각각 ablation flag로 개별 제어 가능하다.

#### 7.6.1 Confidence 가중치

> 📁 `DV_RobustKernels.h:10–14`

$$
w_{\text{conf}}(c) = \begin{cases}
1.0 & c \geq 1.5 \;\text{(ARKit High)} \\
0.5 & c \geq 0.5 \;\text{(ARKit Medium)} \\
0.0 & c < 0.5 \;\text{(ARKit Low → 제거)}
\end{cases}
\tag{47}
$$

iPhone LiDAR의 confidence 값(0, 1, 2)에 기반한 **3단계 가중치**이다. Low confidence 점은 하드 게이트로 즉시 제거되고, Medium은 절반 가중치로 ESKF 업데이트에 기여한다.

#### 7.6.2 Truncated Least Squares (TLS)

> 📁 `DV_RobustKernels.h:17–22`

$$
w_{\text{TLS}}(r) = \begin{cases}
1.0 & |r| \leq \tau_{\text{TLS}} \\
0.0 & |r| > \tau_{\text{TLS}}
\end{cases}, \quad \tau_{\text{TLS}} = 0.10\;\text{m}
\tag{48}
$$

10cm 이상의 잔차를 아웃라이어로 하드 절단한다. Huber나 Cauchy 등의 점진적 감쇠 대신 이진 절단을 사용하는 이유는, 10cm 이상의 점-평면 거리는 맵 불일치(오래된 맵)나 동적 물체에 의한 것이므로 어떠한 가중치로도 유용하지 않기 때문이다.

#### 7.6.3 관측 노이즈 스케일링

> 📁 `src/DV_LIOBackend.cpp:172–174`

최종 관측 노이즈:

$$
\mathbf{R}_{\text{obs}}(i,i) = \frac{\sigma_{\text{base}}^2}{\max(w_{\text{conf}} \cdot w_{\text{TLS}},\; 0.01)}
\tag{49}
$$

$\sigma_{\text{base}} = 0.01\;\text{m}$ (1cm, 하드코딩). 가중치가 높을수록 노이즈가 작아지고, 해당 관측이 ESKF에 더 큰 영향력을 행사한다.

**최소 관측 수:** 유효 관측점 $N < 10$이면 업데이트를 기각한다 (`DV_LIOBackend.cpp:146`).

### 7.7 복셀 해시맵 (Spatial Hash Map)

> 📁 `include/DV_VoxelHashMap.h` (헤더 온리, 275줄)

#### 7.7.1 해시 함수

> 📁 `DV_VoxelHashMap.h:224–229`

$$
h(\mathbf{p}) = \lfloor p_x / s \rfloor + \lfloor p_y / s \rfloor \cdot 10^4 + \lfloor p_z / s \rfloor \cdot 10^8
\tag{50}
$$

$s = 0.1\text{m}$ (10cm). 축별 정수 인덱스에 서로 다른 스케일을 곱하여 1차원 해시로 변환한다. 충돌 없는 범위: 축당 $\pm 5{,}000$ 복셀 (±500m).

> **Remark (GPU 셰이더 해시와의 차이).** GPU 실시간 복셀 중복 제거 (`Shaders.metal:70`)는 XOR 기반 해시를 사용한다: `(ix * 73856093u) ^ (iy * 19349663u) ^ (iz * 83492791u)`. 이는 시각적 중복 방지용이며 SLAM ICP에는 사용되지 않는다.

#### 7.7.2 구조 파라미터

| 파라미터 | 값 | 소스 |
|---------|-----|------|
| 복셀 크기 $s$ | 0.1m | `DV_LIOBackend.cpp:10` |
| 복셀당 최대 점 | 20 | `DV_VoxelHashMap.h:63` |
| 최대 복셀 수 | 500,000 | `DV_LIOBackend.cpp:10` |
| KNN $K$ | 5 | `DV_LIOBackend.cpp:117` |
| 검색 범위 | 27 인접 복셀 ($3^3$) | `DV_VoxelHashMap.h:143–146` |

#### 7.7.3 LRU 제거

> 📁 `DV_VoxelHashMap.h:244–264`

맵이 최대 복셀 수에 도달하면, `partial_sort`로 가장 오래된 10%($= 50{,}000$)를 제거한다. 각 복셀의 `last_access` 카운터를 오름차순 부분 정렬하여 하위 10%를 선택한다.

### 7.8 키프레임 선택

> 📁 `src/DepthVizEngine.cpp:206–223` — `isKeyframe()`

새 키프레임 조건 (OR):

$$
\|\mathbf{p}_k - \mathbf{p}_{k-1}\|_2 \geq 0.05\;\text{m} \quad \lor \quad \arccos\left(\text{clamp}\left(\frac{\text{tr}(\Delta\mathbf{R}) - 1}{2},\; -1,\; 1\right)\right) \geq 2°
\tag{51}
$$

여기서 $\Delta\mathbf{R} = \mathbf{R}_{k-1}^T \mathbf{R}_k$ (`DepthVizEngine.cpp:214`). 키프레임이 아닌 프레임에서는 ICP 최적화와 맵 누적을 건너뛴다 (`DepthVizEngine.cpp:357–366`).

### 7.9 발산 방지 (Divergence Guard)

> 📁 `src/DV_LIOBackend.cpp:62–75`

ESKF 위치가 ARKit prior에서 1m 이상 벗어나면 ARKit 포즈로 **부분 리셋**:

$$
d = \|\mathbf{p}_{\text{ESKF}} - \mathbf{p}_{\text{ARKit}}\|_2
\tag{52}
$$

$$
\text{if}\; d > 1.0\;\text{m}: \quad \mathbf{R}, \mathbf{p} \leftarrow \mathbf{R}_{\text{ARKit}}, \mathbf{p}_{\text{ARKit}}
\tag{53}
$$

**핵심:** 속도 $\mathbf{v}$, 바이어스 $\mathbf{b}_g, \mathbf{b}_a$, 중력 $\mathbf{g}$는 **보존**한다 (`DV_LIOBackend.cpp:70,73`). 이는 발산의 원인이 대부분 포즈 추정의 급격한 이탈이며, 바이어스/중력의 누적 추정값은 여전히 유효하기 때문이다.

**추가 안전장치:**
- IMU $\Delta t > 0.5\text{s}$: 예측 건너뜀 (오래된 데이터, `DV_LIOBackend.cpp:31`)
- ESKF 업데이트 실패: ARKit prior를 대체 출력으로 사용 (`DV_LIOBackend.cpp:185–188`)

---

## 8. Bundle and Discard Preprocessing

> 📁 `src/DepthVizEngine.cpp:116–200` — `bundleAndDiscard()`
> 📁 `include/DV_Types.h:277–281` — `BundleDiscardConfig`

### 8.1 목적

SLAM 엔진 입력 전에 원시 포인트(≈3,000/프레임)를 **품질 기반 적응적 감소**한다. 균일 다운샘플링(복셀 그리드 필터)과 달리, 신뢰도와 밀도를 동시에 고려하여 저품질 영역을 우선 제거한다.

### 8.2 파이프라인

```
입력: 원시 포인트 ~3,000개/프레임
  ↓
[Stage 1] Hard Confidence Gate: confidence == 0 → 즉시 제거          (line 140)
  ↓
[Stage 2] 유효성 필터: NaN 또는 ||p||² > 100 (>10m) → 제거           (line 147-148)
  ↓
[Stage 3] 복셀 해싱: s=0.1m 복셀로 분류, 위치/RGB/confidence 누적    (line 151-167)
  ↓
[Stage 4] Density Gate: 복셀 내 점 < 5개 → 통째 제거                 (line 178)
  ↓
[Stage 5] Avg Confidence Gate: 복셀 평균 confidence < 1.5 → 제거     (line 181-182)
  ↓
[Stage 6] Centroid Emission: 복셀당 1개 평균점 (위치, RGB 모두 평균)   (line 185-196)
  ↓
출력: 번들링된 포인트 ~200-500개/프레임
```

### 8.3 설정값

| 파라미터 | 기호 | 기본값 | 소스 |
|---------|------|--------|------|
| 복셀 크기 | $s_{\text{BD}}$ | 0.1m | `DV_Types.h:278` |
| 최소 밀도 | $n_{\text{min}}$ | 5 | `DV_Types.h:279` |
| 최소 평균 신뢰도 | $\bar{c}_{\text{min}}$ | 1.5 | `DV_Types.h:280` |

**감소율:** $1 - |\mathcal{P}_{\text{bundled}}|/|\mathcal{P}_{\text{raw}}| \approx 80\text{–}95\%$

### 8.4 수학적 정의

**복셀 $v$의 중심점(centroid) 방출:**

$$
\mathbf{p}_v = \frac{1}{n_v}\sum_{i \in v}\mathbf{p}_i, \quad \mathbf{c}_v = \frac{1}{n_v}\sum_{i \in v}\mathbf{c}_i, \quad \bar{c}_v = \frac{1}{n_v}\sum_{i \in v}c_i
\tag{54}
$$

여기서 $n_v$는 복셀 내 점 수, $\mathbf{c}_i = (r_i, g_i, b_i)$는 RGB, $c_i$는 confidence.

**방출 조건:** $n_v \geq n_{\text{min}}$ **AND** $\bar{c}_v \geq \bar{c}_{\text{min}}$

**글로벌 맵 한도:** 키프레임에서만 누적하며, 총 **2,000,000점** (`DepthVizEngine.hpp:151`)을 초과하면 추가 누적을 중단한다.

---

## 9. Post-Processing Pipeline

> 📁 `Renderer.swift:803–1303` — `optimizeAndExport()`

내보내기 시 4단계 후처리를 수행한다. DV-SLAM과 ARKit 모드에서 경로가 다르다.

### 9.1 Phase 1 — SLAM 맵 로드

> 📁 `Renderer.swift:880–931`

C++ 엔진의 `getFullMap()`에서 글로벌 맵을 가져온다. SLAM 맵 크기가 GPU 버퍼(ARKit 실시간 포인트)의 **10% 미만**이면 GPU 버퍼를 유지한다 (`Renderer.swift:900`). GPU 버퍼는 이미 SLAM 보정 포즈(`didUpdatePose`)로 누적되었기 때문에, SLAM 맵이 너무 적은 경우 GPU 데이터가 더 신뢰성 있다.

### 9.2 Phase 2 — Surface Thinning (이중 벽 제거)

> 📁 `Renderer.swift:939–1104` (DV-SLAM 전용)

드리프트로 인한 **이중 벽(double wall)** 문제를 히스토그램 기반으로 해결한다.

**알고리즘:**

1. 포인트를 50mm 셀로 그룹화 (`Renderer.swift:943, 956–963`)
2. 각 셀에서 **최대 확산 축**을 결정 (X, Y, Z 중 bounding box가 가장 큰 축, `Renderer.swift:982–985`)
3. 확산 축을 따라 5mm 빈 히스토그램을 구축 (`Renderer.swift:997–1012`)
4. **갭 감지**: 15mm 이상 연속된 빈 빈(empty bin)이 존재하는지 확인 (`Renderer.swift:1016–1028`). 갭이 없으면 단일 표면(모서리/코너)으로 판단하고 건드리지 않음
5. **슬라이딩 윈도우**: 25mm 윈도우로 가장 밀집된 구간을 탐색 (`Renderer.swift:1031–1048`)
6. 밀집 구간 외부의 점을 제거 대상으로 표시

**파라미터:**

| 파라미터 | 값 | 라인 |
|---------|-----|------|
| 분석 셀 크기 | 50mm | 943 |
| 히스토그램 빈 | 5mm | 944 |
| 최소 스프레드 | 30mm (미만: 단일 표면, skip) | 945 |
| 갭 임계값 | 15mm (연속 빈 빈) | 946 |
| 보존 윈도우 | 25mm | 947 |
| 최대 제거 비율 | 40% (안전장치) | 948 |

### 9.3 Phase 3 — Voxel Downsampling

> 📁 `Renderer.swift:1108–1167`

| 모드 | 복셀 크기 | 라인 |
|------|----------|------|
| DV-SLAM | 12mm | 835 |
| ARKit | 20mm | 848 |

복셀 내 위치와 RGB를 산술 평균하고, confidence는 최대값을 보존한다 (`Renderer.swift:1136`).

### 9.4 Phase 4 — Statistical Outlier Removal (SOR)

> 📁 `Renderer.swift:1175–1303`

**이중 기준 필터링:**

**(A) 밀도 기반** (`Renderer.swift:1213–1228`): 30mm 셀 그리드에서 27-이웃(자신 포함) 합계가 3 미만이면 완전 고립으로 판단하여 제거.

**(B) 거리 기반 IQR** (`Renderer.swift:1230–1245`): 전체 점의 중심점(centroid)까지 거리의 IQR을 계산하고, $Q_3 + 3.0 \times \text{IQR}$ 초과 점을 극단 이상치로 제거.

**안전장치:** 최대 30%까지만 제거 (`Renderer.swift:1178`).

---

## 10. Hybrid ARKit-LIO Fusion

> 📁 `src/DV_VIOManager.cpp`

### 10.1 보정 행렬 계산

> 📁 `DV_VIOManager.cpp:45–58` — `updatePoseFromLIO()`

키프레임마다 LIO 정제 포즈와 ARKit 포즈 간의 보정 변환을 계산한다:

$$
\mathbf{T}_{\text{correction}} = \mathbf{T}_{\text{LIO}} \cdot \mathbf{T}_{\text{ARKit}}^{-1}
\tag{55}
$$

ARKit 역변환은 수동 계산 (`DV_VIOManager.cpp:52–54`):

$$
\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T\mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}
\tag{56}
$$

### 10.2 보정된 포즈 출력

> 📁 `DV_VIOManager.cpp:22–35` — `getPose()`

$$
\mathbf{T}_{\text{corrected}} = \mathbf{T}_{\text{correction}} \cdot \mathbf{T}_{\text{ARKit,new}}
\tag{57}
$$

보정은 키프레임마다 갱신되며, 키프레임 사이의 ARKit 포즈에도 좌측 곱으로 전파된다.

### 10.3 운용 모드 분리

| 경로 | 포즈 소스 | 목적 |
|------|----------|------|
| 실시간 렌더링 | ARKit 원본 (`Renderer.swift:343–345`) | 30fps, 안정성 |
| SLAM 내부 | ESKF + ICP 정제 | 정밀도 |
| 내보내기 | SLAM 글로벌 맵 + 4단계 후처리 | 최종 품질 |
| 발산 시 | ARKit fallback (§7.9) | 안전망 |

---

## 11. Comparison with Apple ARKit

### 11.1 아키텍처 차이

| 항목 | Apple ARKit | DV-SLAM |
|------|-------------|---------|
| **프레임워크** | Visual-Inertial Odometry (VIO) | LiDAR-Inertial Odometry (LIO) |
| **주 센서** | 카메라 (시각적 특징점 매칭) | LiDAR (기하학적 정합) |
| **LiDAR 역할** | 보조 (depth anchor, 메쉬용) | **주 센서** (ICP 관측) |
| **구현** | 비공개 (블랙박스) | 오픈 소스 |

### 11.2 정량적 비교

| 항목 | ARKit | DV-SLAM |
|------|-------|---------|
| **조명 의존성** | 높음 (카메라 기반) | 낮음 (능동 센서) |
| **텍스처 없는 환경** | 특징점 부족 → 드리프트 | 기하학 기반 → 영향 없음 |
| **드리프트 보정** | 루프 클로저 (시각) | ICP + ESKF (기하학) |
| **노이즈 제거** | 기본 confidence 필터 | Bundle & Discard + TLS + SOR |
| **이중 벽 보정** | 없음 | Surface Thinning (§9.2) |
| **내보내기 해상도** | 20mm | 12mm |
| **바이어스 추정** | 내부 (비공개) | 온라인 ESKF ($\mathbf{b}_g, \mathbf{b}_a$) |

### 11.3 ARKit 유리 시나리오

- **실시간 렌더링**: Apple GPU 파이프라인 최적화 (DV-SLAM은 export-only)
- **루프 클로저**: 시각 기반 장소 인식 (DV-SLAM은 미지원)
- **대규모 환경**: ARKit의 세션 관리 및 재위치화(relocalization) 기능

---

## 12. DV-SLAM Novelties

기존 LIO 알고리즘(FAST-LIO2, DLIO, Super-LIO)은 기계식 스피닝 LiDAR(Velodyne VLP-16/128, Livox Avia 등)를 전제로 설계되었다. 이러한 시스템은 (a) 스캔당 $>$100K점의 밀집 포인트클라우드, (b) mm 수준의 거리 정밀도, (c) 넓은 FOV (360° 수평), (d) 고출력 레이저를 가정한다. iPhone dToF LiDAR는 이 모든 가정에서 벗어난다. DV-SLAM의 핵심 차별점을 요약한다.

### 12.1 ARKit Confidence 다중 게이팅

기존 LIO에는 **점별 신뢰도** 개념이 없다. 기계식 LiDAR는 반사 강도(intensity)만 제공하며, 표면 재질 의존적이라 품질 지표로 부적합하다. DV-SLAM은 ARKit confidence를 3단계로 활용한다:

| 레벨 | 위치 | 작용 |
|------|------|------|
| Gate 1 (점 단위) | `DepthVizEngine.cpp:140` | $c = 0$ → 즉시 제거 |
| Gate 2 (복셀 단위) | `DepthVizEngine.cpp:181–182` | $\bar{c} < 1.5$ → 통째 제거 |
| Gate 3 (ICP 단위) | `DV_RobustKernels.h:10–14` | 관측 노이즈 가중치 차등 |

### 12.2 Bundle & Discard

기존 LIO는 균일 다운샘플링(복셀 그리드 필터)을 사용한다. DV-SLAM의 Bundle & Discard는 밀도와 신뢰도를 동시 고려하는 **적응적 감소**로 80–95% 감소율을 달성하면서 고품질 영역의 정보를 보존한다.

### 12.3 Divergence Guard with ARKit Fallback

기존 LIO가 발산하면 복구 불가하다. DV-SLAM은 ARKit 포즈를 상시 수신하며, ESKF와의 차이가 1m을 초과하면 포즈를 리셋하되 바이어스/중력 추정은 보존하는 **부분 리셋** 전략을 사용한다.

### 12.4 Surface Thinning

루프 클로저 없이 **히스토그램 기반 통계적 방법**으로 이중 벽을 제거한다. 갭 감지로 이중 벽과 정상 모서리를 구분한다.

### 12.5 Export-Only Architecture

실시간 렌더링을 ARKit에 위임하고 SLAM을 백그라운드에서 수행함으로써, (a) ICP 3회 반복이 가능하고 (b) 200만 점 글로벌 맵을 유지하며 (c) 4단계 후처리를 수행할 수 있다. 실시간 프레임 드롭은 0이다.

### 12.6 경량 의존성

| 시스템 | 핵심 의존성 | 크기 영향 |
|--------|-----------|----------|
| FAST-LIO2 | PCL, Eigen, ikd-Tree | ~50MB+ |
| DLIO | PCL, nanoflann, TBB | ~50MB+ |
| FAST-LIVO2 | PCL, OpenCV, Eigen | ~100MB+ |
| **DV-SLAM** | **Eigen 헤더 온리** | **~3MB** |

복셀 해시맵, KNN, Lie 군 연산, 강건 커널을 모두 STL + Eigen으로 자체 구현했다.

### 12.7 적응적 누적 임계값

> 📁 `Renderer.swift:43–52`

| 파라미터 | DV-SLAM | ARKit | 소스 |
|---------|---------|-------|------|
| 회전 임계값 | 2° | 5° | `Renderer.swift:44–47` |
| 이동 임계값 | 1.5cm | 3cm | `Renderer.swift:48–52` |
| 그리드 밀도 | 4096점/프레임 | 2048점/프레임 | `Renderer.swift:33–34` |

DV-SLAM은 SLAM 보정으로 밀도를 높여도 안전하므로 더 공격적인 취득 설정을 사용한다.

### 12.8 iPhone dToF 고유 노이즈 보상 전략

iPhone dToF (direct Time-of-Flight) LiDAR는 SPAD (Single-Photon Avalanche Diode) 어레이 기반 센서로, 기계식 LiDAR와 근본적으로 다른 노이즈 특성을 가진다. DV-SLAM은 이러한 dToF 고유 한계에 특화된 보상 전략을 구현한다.

#### 12.8.1 dToF 양자화 노이즈와 거리 의존적 정밀도

| 파라미터 | iPhone dToF | 기계식 LiDAR (VLP-16) | 영향 |
|---------|-------------|---------------------|------|
| 거리 정밀도 (1m) | ±1cm | ±3mm | dToF 3배 낮음 |
| 거리 정밀도 (5m) | ±5–8cm | ±3mm | dToF 20배+ 낮음 |
| 최대 범위 | ~5m | 100m | dToF 1/20 |
| 포인트 수/프레임 | ~49K (256×192) | ~300K (16채널) | dToF 1/6 |
| SLAM 입력/프레임 | ~200–500 (B&D 후) | ~30K (다운샘플 후) | dToF 1/60–1/150 |
| FOV | ~60° (전방) | 360° × 30° | dToF 극히 좁음 |
| 멀티패스 간섭 | 높음 (반사면) | 낮음 | dToF 고유 문제 |

**DV-SLAM의 대응:**

1. **TLS 임계값 $\tau = 0.10$m** (Eq. 48): 기계식 LiDAR 시스템(FAST-LIO2: $\tau \approx 0.03$m)보다 3배 관대한 임계값은 dToF의 거리 의존적 양자화 노이즈를 수용한다. 5m 거리에서 ±5cm 노이즈가 발생하므로 10cm 이내의 잔차는 정상 범위로 간주한다.

2. **관측 노이즈 $\sigma_{\text{base}} = 0.01$m** (Eq. 49): 기계식 LiDAR 대비 보수적 설정. 낮은 가중치의 관측이 ESKF를 과도하게 끌어가는 것을 방지한다.

3. **ARKit Confidence → 관측 노이즈 역결합** (Eq. 49): $w_{\text{conf}} = 0.5$ (Medium)일 때 관측 노이즈가 $4\sigma_{\text{base}}^2$로 4배 증가. 이는 dToF의 저신뢰 측정이 ESKF에 미치는 영향을 명시적으로 감쇠시킨다.

#### 12.8.2 극단적 희소성에서의 ICP 안정성

프레임당 ~200–500점으로 ICP를 수행하는 것은 기계식 LiDAR 시스템(~30K점)과 비교하여 **60–150배** 적은 관측이다. 이 극단적 희소성에서 ICP의 안정성을 확보하기 위한 DV-SLAM의 전략:

1. **최소 관측 임계값 $N_{\min} = 10$** (`DV_LIOBackend.cpp:146`): 유효 관측점이 10개 미만이면 ESKF 업데이트를 기각하고 ARKit prior를 사용한다. 이는 underconstrained 최적화로 인한 발산을 원천 차단한다.

2. **넓은 복셀 검색 범위 ($3^3 = 27$ 인접 복셀)**: 희소한 맵에서도 충분한 KNN 이웃을 확보하기 위해 27개 인접 복셀을 탐색한다. 기계식 LiDAR 시스템에서는 맵 밀도가 충분하여 1 복셀 검색만으로도 충분한 경우가 많다.

3. **보수적 평면성 검증 ($\lambda_0/\lambda_1 < 0.3$)** (Eq. 44): K=5개의 적은 이웃에서 추정한 평면의 신뢰도가 낮을 수 있으므로, 30%의 관대한 임계값으로 비평면 구조도 일부 허용한다.

4. **IEKF 3회 반복**: 소수 관측에서의 선형화 오차를 반복 업데이트로 보상한다. 1회 업데이트만으로는 희소 관측의 비선형성이 충분히 흡수되지 않는다.

#### 12.8.3 핸드헬드 모션 모델과 보수적 프로세스 노이즈

iPhone은 핸드헬드 기기이므로, 로봇/자율주행 차량에 장착된 기계식 LiDAR와는 근본적으로 다른 모션 프로파일을 가진다:

| 특성 | 로봇 장착 | iPhone 핸드헬드 |
|------|----------|----------------|
| 최대 각속도 | ~0.5 rad/s | ~3 rad/s (급격한 손 회전) |
| 진동/떨림 | 저주파 (차체) | 고주파 (손 떨림, 1-10Hz) |
| 가속도 변동 | 완만 | 급격 (팔 동작) |
| 동작 예측 가능성 | 높음 (제어 입력 알려짐) | 낮음 (비구조적 움직임) |

**DV-SLAM의 대응:**

1. **보수적 위치 프로세스 노이즈** $\mathbf{Q}_p = \sigma_a^2 \Delta t^3/4$: 엄밀한 이산화($\sigma_a^2 \Delta t^5/20$)보다 $\approx 50{,}000$배 큰 값. 핸드헬드 모션의 불확실성을 과대 추정하여 필터가 LiDAR 관측에 더 많이 의존하게 유도한다 (§6.3 Remark 참조).

2. **중점 적분 (Eq. 26–27)**: 1차 Euler 적분 대신 $\mathbf{R}_{\text{mid}}$를 사용하여, 급격한 회전 시 가속도 변환의 정확도를 높인다. $\|\boldsymbol{\omega}\| = 3$ rad/s일 때, 중점 적분은 Euler 대비 $\Delta t \times 3/2 = 0.015$ rad의 회전 오차를 절반으로 줄인다.

3. **키프레임 선택 (Eq. 51)의 이중 기준**: 핸드헬드 모션에서 이동 없이 회전만 하는 경우(제자리 팬)가 빈번하므로, 이동 **또는** 회전 중 하나만 만족해도 키프레임을 생성한다.

#### 12.8.4 미소 레버암 가정

> 📁 `src/DV_LIOBackend.cpp:107–110`

iPhone에서 LiDAR 센서와 IMU 간 물리적 거리(레버암)는 $\ell \approx 5\text{–}10$mm이다:

$$
\|\mathbf{T}_{\text{cam-imu}} - \mathbf{I}_4\|_F \approx \ell \leq 0.01\text{m}
$$

이로 인한 최대 위치 편향:

$$
\epsilon_{\text{lever}} = \|\boldsymbol{\omega}\| \times \ell \leq 3 \times 0.01 = 0.03\text{m}
$$

일반적 핸드헬드 동작($\|\boldsymbol{\omega}\| \leq 1$ rad/s)에서는 $\epsilon_{\text{lever}} \leq 1$cm이다. 이는 dToF 양자화 노이즈(1–5cm)보다 작거나 동등하므로, **외부 캘리브레이션을 생략하는 것이 정당화된다**. 기계식 LiDAR 시스템에서는 레버암이 10–30cm이므로 반드시 정밀 캘리브레이션이 필요하다.

> **Remark.** 이 가정은 DV-SLAM이 iPhone 이외의 플랫폼(iPad Pro 포함)으로 이식될 때 재검증해야 한다. iPad Pro의 LiDAR-IMU 거리는 iPhone보다 크며, 레버암 오차가 dToF 노이즈를 초과할 수 있다.

### 12.9 RGB 색상 보존 파이프라인

```
ARFrame.capturedImage (YCbCr NV12)
  → BT.601 변환 → RGB                      (SLAMService.mm:449–454)
  → DVPoint3D{x,y,z,r,g,b,confidence}      (DV_Types.h:42–72)
  → Bundle & Discard: 복셀 내 RGB 평균       (DepthVizEngine.cpp:185–196)
  → 글로벌 맵 누적: RGB 보존                  (DepthVizEngine.cpp:394–413)
  → Voxel Downsampling: 복셀 내 RGB 평균     (Renderer.swift:1116–1157)
  → PLY 출력: vertex color 포함
```

---

## 13. Computational Complexity

| 모듈 | 시간 복잡도 | 비고 |
|------|------------|------|
| IMU 예측 | $O(1)$ per sample | 18×18 행렬 곱 |
| Bundle & Discard | $O(N)$ | $N$: 원시 점 수, 해시맵 삽입 |
| KNN 검색 | $O(27 \times 20) = O(540)$ per point | 고정 복셀 검색 범위 |
| 평면 추정 | $O(K^2)$, $K=5$ | 3×3 고유값 분해 |
| ICP 관측 함수 | $O(M \times 540)$ | $M$: 번들링 후 점 수 |
| ESKF 업데이트 | $O(M \times 18^2 + 18^3)$ | $\mathbf{H}$ 구성 + LDLT |
| IEKF 반복 | $O(3 \times \text{위})$ | 최대 3회 |
| Surface Thinning | $O(N_{\text{map}})$ | 히스토그램 구축 + 스캔 |
| SOR | $O(N_{\text{map}} \log N_{\text{map}})$ | 거리 정렬 |
| 전체 파이프라인 | $O(N_{\text{map}} \log N_{\text{map}})$ | SOR이 지배적 |

---

## Appendix A. Algorithm Pseudocode

### Algorithm 1: DV-SLAM Main Loop

```
Input: LiDAR 깊이맵 스트림, IMU 데이터 스트림, ARKit 포즈 스트림
Output: 글로벌 포인트클라우드 맵 M

Initialize: ESKF state x₀, voxel hash map V, full_map M ← ∅

For each LiDAR frame (timestamp t):
  1. Unproject depth → raw points P_raw (~3,000 pts)
  2. P_bundled ← BundleAndDiscard(P_raw)                    [§8]
  3. Drain IMU buffer: for each IMU sample with τ ≤ t:
       ESKF.predict(imu, Δt)                                [§6.2]
  4. T_prior ← ARKit pose
  5. If DivergenceGuard(ESKF.p, T_prior.p, τ=1.0m):        [§7.9]
       Reset ESKF.R, ESKF.p ← T_prior (preserve v,b,g)
  6. If NOT isKeyframe(T_prior):                            [§7.8]
       Continue (skip ICP)
  7. Build observation function:
       For each p ∈ P_bundled:
         p_world ← R·p + t                                 [Eq.41]
         KNN ← V.getTopK(p_world, K=5)                     [§7.3.1]
         (n, q̄) ← fitPlane(KNN)                            [§7.3.2]
         r ← nᵀ(p_world - q̄)                              [Eq.45]
         w ← ConfidenceWeight(p.conf) × TLSWeight(r)       [§7.6]
         Accumulate H_i, r_i, R_obs(i,i)                   [Eq.46,49]
  8. T_refined ← ESKF.updateObserve(obs_func)               [§6.5]
  9. V.insert(T_refined, P_bundled)
  10. M ← M ∪ Transform(P_bundled, T_refined)
  11. T_correction ← T_refined · T_prior⁻¹                 [Eq.55]

On Export:
  12. Phase 1: Load SLAM map (or keep GPU buffer if <10%)   [§9.1]
  13. Phase 2: Surface Thinning (DV-SLAM only)              [§9.2]
  14. Phase 3: Voxel Downsampling (12mm / 20mm)             [§9.3]
  15. Phase 4: Statistical Outlier Removal                  [§9.4]
  16. Export to PLY (ASCII or binary)
```

---

## Appendix B. Parameter Table

### B.1 ESKF Parameters (`DV_Types.h:291–301`)

| 파라미터 | 기호 | 값 | 단위 |
|---------|------|-----|------|
| IEKF 반복 횟수 | $N_{\text{iter}}$ | 3 | — |
| 수렴 임계값 | $\epsilon_{\text{quit}}$ | $10^{-6}$ | — |
| 자이로 노이즈 | $\sigma_g$ | 0.01 | rad/s/$\sqrt{\text{Hz}}$ |
| 가속도 노이즈 | $\sigma_a$ | 0.1 | m/s²/$\sqrt{\text{Hz}}$ |
| 자이로 바이어스 노이즈 | $\sigma_{bg}$ | 0.001 | rad/s²/$\sqrt{\text{Hz}}$ |
| 가속도 바이어스 노이즈 | $\sigma_{ba}$ | 0.01 | m/s³/$\sqrt{\text{Hz}}$ |
| LiDAR 관측 노이즈 | $\sigma_{\text{lidar}}$ | 0.01 | m |

### B.2 Spatial Parameters

| 파라미터 | 값 | 소스 |
|---------|-----|------|
| ICP 복셀 크기 | 0.1m | `DV_LIOBackend.cpp:10` |
| 최대 복셀 수 | 500,000 | `DV_LIOBackend.cpp:10` |
| 복셀당 최대 점 | 20 | `DV_VoxelHashMap.h:63` |
| KNN K | 5 | `DV_LIOBackend.cpp:117` |
| 평면성 임계값 | $\lambda_0/\lambda_1 < 0.3$ | `DV_VoxelHashMap.h:194` |
| TLS 임계값 | 0.10m | `DV_RobustKernels.h:17` |
| 발산 임계값 | 1.0m | `DV_LIOBackend.cpp:68` |

### B.3 Keyframe Parameters (`DV_Types.h:283–289`)

| 파라미터 | 값 |
|---------|-----|
| 이동 임계값 | 0.05m (5cm) |
| 회전 임계값 | 2° (≈0.0349 rad) |

### B.4 Bundle & Discard Parameters (`DV_Types.h:277–281`)

| 파라미터 | 값 |
|---------|-----|
| 복셀 크기 | 0.1m |
| 최소 밀도 | 5 |
| 최소 평균 신뢰도 | 1.5 |

### B.5 Post-Processing Parameters (`Renderer.swift`)

| Phase | 파라미터 | 값 | 라인 |
|-------|---------|-----|------|
| Phase 2 | 분석 셀 | 50mm | 943 |
| Phase 2 | 히스토그램 빈 | 5mm | 944 |
| Phase 2 | 갭 임계값 | 15mm | 946 |
| Phase 2 | 보존 윈도우 | 25mm | 947 |
| Phase 2 | 최대 제거 | 40% | 948 |
| Phase 3 | DV-SLAM 복셀 | 12mm | 835 |
| Phase 3 | ARKit 복셀 | 20mm | 848 |
| Phase 4 | SOR 셀 크기 | 30mm | 1182 |
| Phase 4 | 고립 임계값 | 이웃 < 3 | 1273 |
| Phase 4 | IQR 배수 | 3.0 | 1245 |
| Phase 4 | 최대 제거 | 30% | 1178 |

### B.6 Accumulation Thresholds (`Renderer.swift:43–52`)

| 파라미터 | DV-SLAM | ARKit |
|---------|---------|-------|
| 회전 임계값 | 2° | 5° |
| 이동 임계값 | 1.5cm | 3cm |
| 그리드 밀도 | 4096점 | 2048점 |
| 글로벌 맵 한도 | 2,000,000점 | — |

---

## Appendix C. Source File Index

| 파일 | 위치 | 핵심 내용 | 줄 수 |
|------|------|----------|------|
| `DV_Types.h` | `include/` | SO(3)/SE(3) Lie 군, SysState(18D), DVPoint3D, 설정 구조체 | 305 |
| `DV_ESKF.h/.cpp` | `include/`, `src/` | 18D ESKF: predict, iterated update, Joseph form, buildProcessNoise | 178 |
| `DV_LIOBackend.h/.cpp` | `include/`, `src/` | Point-to-Plane ICP, 관측 야코비안, divergence guard | 258 |
| `DV_VoxelHashMap.h` | `include/` | 공간 해시맵, approximate KNN(K=5), 평면 추정, LRU 제거 | 275 |
| `DV_RobustKernels.h` | `include/` | Confidence 가중치, TLS | 27 |
| `DepthVizEngine.hpp/.cpp` | `include/`, `src/` | Bundle & Discard, 키프레임 검출, 메인 루프, 프로파일링 | 426 |
| `DV_VIOManager.h/.cpp` | `include/`, `src/` | ARKit 포즈 관리, T_correction 계산 | 59 |
| `SLAMService.h/.mm` | `Bridge/` | Swift↔C++ 브릿지, 깊이 역투영, YCbCr→RGB, IMU 변환 | ~500 |
| `Renderer.swift` | `Domain/` | Metal 렌더링, IMU 수집, 후처리 4단계, flipYZ | ~1380 |
| `Shaders.metal` | `Domain/Metal/` | GPU 역투영 (`worldPoint`), 복셀 중복 제거, YCbCr→RGB | ~130 |

> **경로 규칙:** SLAM 코어 파일은 `DepthViz/Domain/Algorithm/DepthViz/` 기준, `SLAMService`는 `Bridge/`, `Renderer.swift`/`Shaders.metal`은 `DepthViz/Domain/`.

---

## Appendix D. Related Work

DV-SLAM은 다음 오픈소스 알고리즘의 수학적 기반을 참고하여 iOS에 최적화하였다:

| 알고리즘 | 기관 | 핵심 기여 | 참고 |
|---------|------|----------|------|
| FAST-LIO2 [1] | HKU | ikd-Tree, IKFoM manifold ESKF, $S^2$ 중력 | Xu et al., IEEE T-RO 2022 |
| FAST-LIVO2 [2] | HKU | Voxel octree, 패치 기반 시각 추적 | Zheng et al., IEEE T-RO 2024 |
| DLIO [3] | UCLA VECTR | NanoGICP, 기하학적 관측기 | Chen et al., RA-L 2023 |
| Super-LIO [4] | ECUST | OctVoxMap, SMW 업데이트 | Wang et al., 2024 |

**DV-SLAM의 차별점:** ESKF + Point-to-Plane ICP의 핵심 프레임워크를 유지하면서, ARKit confidence 다중 게이팅, Bundle & Discard, Surface Thinning, Divergence Guard 등 **iPhone dToF LiDAR 특화 모듈**을 추가하였다. 의존성도 Eigen 헤더 온리로 최소화하여 모바일 환경에 적합하게 설계하였다.

---

## References

[1] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry," IEEE Transactions on Robotics, vol. 38, no. 4, pp. 2053–2073, 2022.

[2] C. Zheng et al., "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry," IEEE Transactions on Robotics, vol. 40, pp. 3070–3088, 2024.

[3] K. Chen et al., "Direct LiDAR-Inertial Odometry and Mapping: Perceptive and Connective SLAM," IEEE Robotics and Automation Letters, vol. 8, no. 8, pp. 4714–4721, 2023.

[4] X. Wang et al., "Super-LIO: Super-Resolution LiDAR-Inertial Odometry," in Proc. IEEE/RSJ IROS, 2024.
