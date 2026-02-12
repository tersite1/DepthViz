# DV-SLAM: Mathematical Foundation

## Table of Contents
1. [SLAM 기초 이론](#1-slam-기초-이론)
2. [좌표계와 변환](#2-좌표계와-변환)
3. [센서 입력 파이프라인](#3-센서-입력-파이프라인)
4. [Error-State Kalman Filter (ESKF)](#4-error-state-kalman-filter-eskf)
5. [LiDAR-Inertial Odometry (LIO) Backend](#5-lidar-inertial-odometry-lio-backend)
6. [포인트클라우드 전처리: Bundle & Discard](#6-포인트클라우드-전처리-bundle--discard)
7. [포스트프로세싱 파이프라인](#7-포스트프로세싱-파이프라인)
8. [Apple ARKit 대비 DV-SLAM 이점](#8-apple-arkit-대비-dv-slam-이점)

---

## 1. SLAM 기초 이론

### 1.1 SLAM이란?

**SLAM** (Simultaneous Localization and Mapping)은 미지의 환경에서 센서 데이터만으로 **자신의 위치를 추정**하면서 동시에 **주변 환경의 지도를 구축**하는 문제이다.

수학적으로, 시간 $t$에서의 상태를 다음과 같이 정의한다:

$$
\mathbf{x}_t = \begin{bmatrix} \mathbf{R}_t \\ \mathbf{p}_t \\ \mathbf{v}_t \end{bmatrix} \in SE(3) \times \mathbb{R}^3
$$

여기서:
- $\mathbf{R}_t \in SO(3)$: 3D 회전 행렬 (디바이스 방향)
- $\mathbf{p}_t \in \mathbb{R}^3$: 월드 좌표계 위치
- $\mathbf{v}_t \in \mathbb{R}^3$: 속도 벡터

### 1.2 SLAM의 두 가지 핵심 문제

**Localization (위치 추정):**
센서 관측 $\mathbf{z}_{1:t}$와 제어 입력 $\mathbf{u}_{1:t}$가 주어졌을 때, 상태의 사후 확률을 추정:

$$
p(\mathbf{x}_t \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})
$$

**Mapping (지도 구축):**
추정된 위치를 기반으로 환경의 3D 표현을 구축:

$$
\mathcal{M} = \{(\mathbf{p}_i, \mathbf{c}_i, \sigma_i)\}_{i=1}^{N}
$$

여기서 $\mathbf{p}_i$는 점의 좌표, $\mathbf{c}_i$는 색상 (RGB), $\sigma_i$는 신뢰도(confidence)이다.

### 1.3 LiDAR-Inertial Odometry (LIO)

DV-SLAM은 **LIO** 방식을 채택한다. LiDAR와 IMU를 결합하면:

| 센서 | 장점 | 단점 |
|------|------|------|
| **LiDAR** | 정밀한 깊이 (mm급), 조명 불변 | 프레임 간 대응점 찾기 어려움, 저주파 (30Hz) |
| **IMU** | 초고주파 (100Hz), 순간 자세 변화 캡처 | 드리프트 누적 (적분 오차) |

LIO는 IMU의 고주파 예측을 LiDAR의 정밀 관측으로 **보정(correction)** 하는 상보적 구조이다.

---

## 2. 좌표계와 변환

### 2.1 리 군(Lie Group)과 리 대수(Lie Algebra)

3D 회전은 **SO(3)** 리 군에 속한다. DV-SLAM은 Rodrigues 공식을 사용하여 리 대수 $\mathfrak{so}(3)$와 리 군 $SO(3)$ 사이를 변환한다.

**지수 사상 (Exponential Map)** — 축-각도 벡터 $\boldsymbol{\omega}$를 회전 행렬로:

$$
\text{Exp}(\boldsymbol{\omega}) = \mathbf{I} + \frac{\sin\theta}{\theta}[\boldsymbol{\omega}]_\times + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times^2
$$

여기서 $\theta = \|\boldsymbol{\omega}\|$, $[\boldsymbol{\omega}]_\times$는 반대칭 행렬 (hat operator):

$$
[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}
$$

**로그 사상 (Logarithmic Map)** — 회전 행렬을 축-각도로:

$$
\text{Log}(\mathbf{R}) = \frac{\theta}{2\sin\theta}(\mathbf{R} - \mathbf{R}^T), \quad \theta = \arccos\left(\frac{\text{tr}(\mathbf{R}) - 1}{2}\right)
$$

DV-SLAM 구현은 $\theta \approx \pi$ 근방에서의 수치 안정성을 위해 고유값 분해 기반의 특수 분기를 포함한다.

### 2.2 SE(3) 변환

강체 변환 $\mathbf{T} \in SE(3)$:

$$
\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0}^T & 1 \end{bmatrix}, \quad \text{Exp}(\boldsymbol{\xi}) = \begin{bmatrix} \text{Exp}(\boldsymbol{\omega}) & \mathbf{J}\boldsymbol{\rho} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

여기서 $\boldsymbol{\xi} = [\boldsymbol{\rho}; \boldsymbol{\omega}] \in \mathfrak{se}(3)$이고, **좌측 야코비안(Left Jacobian)**:

$$
\mathbf{J} = \mathbf{I} + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times + \frac{\theta - \sin\theta}{\theta^3}[\boldsymbol{\omega}]_\times^2
$$

### 2.3 DV-SLAM 좌표계 변환 체인

```
카메라 프레임            ARKit 월드 프레임          SLAM 월드 프레임
(x→우, y→하, z→전방)  →  (x→우, y→상, z→후방)   →  (ESKF 최적화 좌표)
       ↕                        ↕                        ↕
  핀홀 역투영              flipYZ 변환              T_correction
  K^{-1} · depth         [1,0,0; 0,-1,0; 0,0,-1]    T_lio · T_arkit^{-1}
```

**flipYZ 변환** (카메라 → ARKit 좌표계):

$$
\mathbf{M}_\text{flipYZ} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & -1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

이 변환이 flipZ가 아닌 **flipYZ**여야 하는 이유: ARKit은 Y=중력 반대 방향(위), Z=카메라 뒤쪽이다. 카메라 내부 좌표계는 Y=아래, Z=전방이므로 Y와 Z 모두 반전해야 한다.

---

## 3. 센서 입력 파이프라인

### 3.1 LiDAR 깊이 데이터

ARKit의 `sceneDepth`는 `256×192` 해상도의 Float32 깊이맵을 제공한다. DV-SLAM은 이를 **핀홀 카메라 모델**로 3D 역투영한다.

**핀홀 역투영 (Unprojection):**

$$
\mathbf{p}_\text{cam} = \begin{bmatrix} \frac{(u - c_x) \cdot d}{f_x} \\ \frac{(v - c_y) \cdot d}{f_y} \\ d \end{bmatrix}
$$

여기서:
- $(u, v)$: 깊이맵 픽셀 좌표
- $d$: 해당 픽셀의 깊이값 (미터)
- $(f_x, f_y)$: 초점 거리 (깊이맵 해상도로 스케일링)
- $(c_x, c_y)$: 주점(principal point)

**내부 파라미터 스케일링:**

카메라 내부 행렬은 캡처 이미지(1920×1440) 해상도 기준이므로 깊이맵 해상도로 변환:

$$
f_x' = \frac{f_x \cdot W_\text{depth}}{W_\text{cam}}, \quad c_x' = \frac{c_x \cdot W_\text{depth}}{W_\text{cam}}
$$

**서브샘플링:** 성능을 위해 4픽셀마다 1개만 처리 (`step=4`), 결과적으로 프레임당 약 3,000개 점을 SLAM 엔진에 전달.

### 3.2 색상 (RGB) 데이터

캡처 이미지는 **YCbCr 4:2:0** 포맷이다. BT.601 변환 행렬로 RGB 변환:

$$
\begin{bmatrix} R \\ G \\ B \end{bmatrix} = \begin{bmatrix} 1 & 0 & 1.402 \\ 1 & -0.344 & -0.714 \\ 1 & 1.772 & 0 \end{bmatrix} \begin{bmatrix} Y \\ C_b - 128 \\ C_r - 128 \end{bmatrix}
$$

색상은 각 3D 점에 바인딩되어 `Point3D{x, y, z, r, g, b, confidence}` 구조체로 SLAM 엔진에 전달된다. 이 색상은 보셀 평균화와 최종 PLY 출력까지 보존된다.

### 3.3 IMU 데이터

`CMMotionManager`에서 **100Hz**로 수집:

**가속도 (Accelerometer):**

$$
\mathbf{a}_\text{raw} = (\mathbf{a}_\text{gravity} + \mathbf{a}_\text{user}) \times 9.81 \;\text{m/s}^2
$$

CoreMotion은 중력과 사용자 가속도를 분리 제공하나, SLAM 엔진에는 합산된 전체 가속도를 전달한다. 이는 ESKF가 중력 벡터를 상태에 포함하여 직접 추정하기 때문이다.

**자이로스코프 (Gyroscope):**

$$
\boldsymbol{\omega}_\text{raw} = [\omega_x, \omega_y, \omega_z]^T \;\text{rad/s}
$$

회전 속도는 바이어스 보정 전 원시값으로 전달되며, ESKF가 자이로 바이어스 $\mathbf{b}_g$를 온라인으로 추정한다.

**IMU 노이즈 모델:**

$$
\boldsymbol{\omega}_\text{meas} = \boldsymbol{\omega}_\text{true} + \mathbf{b}_g + \mathbf{n}_g, \quad \mathbf{n}_g \sim \mathcal{N}(\mathbf{0}, \sigma_g^2 \mathbf{I})
$$

$$
\mathbf{a}_\text{meas} = \mathbf{a}_\text{true} + \mathbf{b}_a + \mathbf{n}_a, \quad \mathbf{n}_a \sim \mathcal{N}(\mathbf{0}, \sigma_a^2 \mathbf{I})
$$

바이어스는 랜덤 워크로 모델링:

$$
\dot{\mathbf{b}}_g \sim \mathcal{N}(\mathbf{0}, \sigma_{bg}^2 \mathbf{I}), \quad \dot{\mathbf{b}}_a \sim \mathcal{N}(\mathbf{0}, \sigma_{ba}^2 \mathbf{I})
$$

---

## 4. Error-State Kalman Filter (ESKF)

### 4.1 상태 벡터

DV-SLAM의 ESKF는 **18차원** 상태 벡터를 사용한다:

$$
\mathbf{x} = \begin{bmatrix} \mathbf{R} \\ \mathbf{p} \\ \mathbf{v} \\ \mathbf{b}_g \\ \mathbf{b}_a \\ \mathbf{g} \end{bmatrix} \in SO(3) \times \mathbb{R}^{15}
$$

| 상태 | 차원 | 설명 |
|------|------|------|
| $\mathbf{R}$ | 3 (SO(3)) | 월드 기준 디바이스 회전 |
| $\mathbf{p}$ | 3 | 월드 좌표 위치 |
| $\mathbf{v}$ | 3 | 월드 좌표 속도 |
| $\mathbf{b}_g$ | 3 | 자이로스코프 바이어스 |
| $\mathbf{b}_a$ | 3 | 가속도계 바이어스 |
| $\mathbf{g}$ | 3 | 중력 벡터 (초기값 $[0, 0, -9.81]^T$) |

**오차 상태 벡터** $\delta\mathbf{x} \in \mathbb{R}^{18}$:

$$
\delta\mathbf{x} = \begin{bmatrix} \delta\boldsymbol{\theta} \\ \delta\mathbf{p} \\ \delta\mathbf{v} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{g} \end{bmatrix}
$$

### 4.2 예측 단계 (Prediction)

IMU 입력이 들어올 때마다 **중점 적분(Midpoint Integration)** 으로 상태를 전파한다:

**바이어스 보정:**

$$
\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_\text{meas} - \mathbf{b}_g, \quad \hat{\mathbf{a}} = \mathbf{a}_\text{meas} - \mathbf{b}_a
$$

**회전 업데이트:**

$$
\Delta\mathbf{R} = \text{Exp}(\hat{\boldsymbol{\omega}} \cdot \Delta t)
$$

$$
\mathbf{R}_{k+1} = \mathbf{R}_k \cdot \Delta\mathbf{R}
$$

**중점 회전** (적분 정밀도 향상):

$$
\mathbf{R}_\text{mid} = \mathbf{R}_k \cdot \text{Exp}(\hat{\boldsymbol{\omega}} \cdot \Delta t / 2)
$$

**월드 좌표 가속도:**

$$
\mathbf{a}_\text{world} = \mathbf{R}_\text{mid} \cdot \hat{\mathbf{a}} + \mathbf{g}
$$

**위치 및 속도 업데이트:**

$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \cdot \Delta t + \frac{1}{2}\mathbf{a}_\text{world} \cdot \Delta t^2
$$

$$
\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_\text{world} \cdot \Delta t
$$

### 4.3 상태 전이 행렬

오차 상태의 전파를 위한 $18 \times 18$ 야코비안 $\mathbf{F}$:

$$
\mathbf{F} = \begin{bmatrix}
\Delta\mathbf{R}^T & \mathbf{0} & \mathbf{0} & -\mathbf{I}\Delta t & \mathbf{0} & \mathbf{0} \\
-\mathbf{R}[\hat{\mathbf{a}}]_\times \frac{\Delta t^2}{2} & \mathbf{I} & \mathbf{I}\Delta t & \mathbf{0} & -\mathbf{R}\frac{\Delta t^2}{2} & \mathbf{I}\frac{\Delta t^2}{2} \\
-\mathbf{R}[\hat{\mathbf{a}}]_\times \Delta t & \mathbf{0} & \mathbf{I} & \mathbf{0} & -\mathbf{R}\Delta t & \mathbf{I}\Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

**공분산 전파:**

$$
\mathbf{P}_{k+1|k} = \mathbf{F} \cdot \mathbf{P}_{k|k} \cdot \mathbf{F}^T + \mathbf{Q}
$$

**프로세스 노이즈 $\mathbf{Q}$** (대각 블록):

$$
\mathbf{Q} = \text{diag}\left(\sigma_g^2\Delta t, \; \sigma_a^2\frac{\Delta t^3}{4}, \; \sigma_a^2\Delta t, \; \sigma_{bg}^2\Delta t, \; \sigma_{ba}^2\Delta t, \; 10^{-10}\right) \otimes \mathbf{I}_3
$$

중력 블록의 노이즈 $10^{-10}$은 준정적 상수 취급을 의미한다.

### 4.4 업데이트 단계 (Update)

LiDAR 관측이 도착하면, **반복적(iterated) 업데이트**를 수행한다 (최대 3회):

**혁신 공분산:**

$$
\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}_\text{obs}
$$

**칼만 이득 (LDLT 분해 사용):**

$$
\mathbf{K} = \mathbf{P}\mathbf{H}^T \mathbf{S}^{-1}
$$

**오차 상태 보정:**

$$
\delta\mathbf{x} = \mathbf{K} \cdot \mathbf{r}
$$

**Joseph 형태 공분산 업데이트** (수치 안정성 보장):

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}_\text{obs}\mathbf{K}^T
$$

**수렴 조건:**

$$
\|\delta\mathbf{x}\| < \epsilon_\text{quit} = 10^{-6}
$$

---

## 5. LiDAR-Inertial Odometry (LIO) Backend

### 5.1 Point-to-Plane ICP

각 LiDAR 점에 대해 맵에서 가장 가까운 **평면**을 찾고, 점-평면 거리를 최소화한다.

**월드 좌표 변환:**

$$
\mathbf{p}_\text{world} = \mathbf{R} \cdot \mathbf{p}_\text{cam} + \mathbf{t}
$$

**최근접 이웃 탐색:** 복셀 해시맵의 $3 \times 3 \times 3 = 27$개 인접 복셀에서 5개 최근접점을 검색.

**국부 평면 추정 (SVD):**

$k$개 이웃점 $\{\mathbf{q}_i\}$에 대해:

$$
\bar{\mathbf{q}} = \frac{1}{k}\sum_{i=1}^{k}\mathbf{q}_i \quad \text{(centroid)}
$$

$$
\mathbf{C} = \frac{1}{k}\sum_{i=1}^{k}(\mathbf{q}_i - \bar{\mathbf{q}})(\mathbf{q}_i - \bar{\mathbf{q}})^T \quad \text{(3×3 공분산 행렬)}
$$

고유값 분해: $\mathbf{C} = \mathbf{U}\boldsymbol{\Lambda}\mathbf{U}^T$

- 법선 벡터: $\mathbf{n}$ = 최소 고유값에 대응하는 고유벡터
- **평면성 검증:** $\lambda_\text{min} / \lambda_\text{mid} < 0.3$ 일 때만 유효 평면

**점-평면 잔차:**

$$
r_i = \mathbf{n}^T (\mathbf{p}_{\text{world},i} - \bar{\mathbf{q}})
$$

### 5.2 관측 야코비안

ESKF 업데이트를 위한 관측 모델의 야코비안 $\mathbf{H} \in \mathbb{R}^{N \times 18}$:

$$
\mathbf{H}_i = \begin{bmatrix} \underbrace{-\mathbf{n}^T \mathbf{R} [\mathbf{p}_\text{cam}]_\times}_{\partial r / \partial \delta\boldsymbol{\theta}} & \underbrace{\mathbf{n}^T}_{\partial r / \partial \delta\mathbf{p}} & \underbrace{\mathbf{0}_{1\times3}}_{\partial r / \partial \delta\mathbf{v}} & \underbrace{\mathbf{0}_{1\times3}}_{\partial r / \partial \delta\mathbf{b}_g} & \underbrace{\mathbf{0}_{1\times3}}_{\partial r / \partial \delta\mathbf{b}_a} & \underbrace{\mathbf{0}_{1\times3}}_{\partial r / \partial \delta\mathbf{g}} \end{bmatrix}
$$

LiDAR 관측은 회전과 위치만 직접 관측 가능하며, 속도·바이어스·중력은 IMU 예측 모델을 통해 간접적으로 추정된다.

### 5.3 강건 가중치 (Robust Weighting)

DV-SLAM은 두 가지 강건 커널을 적용한다:

**Confidence 가중치:**

$$
w_\text{conf} = \begin{cases} 1.0 & \text{confidence} \geq 1.5 \;\text{(High)} \\ 0.5 & \text{confidence} \geq 0.5 \;\text{(Medium)} \\ 0.0 & \text{confidence} < 0.5 \;\text{(Low → 제거)} \end{cases}
$$

**Truncated Least Squares (TLS):**

$$
w_\text{TLS} = \begin{cases} 1.0 & |r_i| \leq 0.10\;\text{m} \\ 0.0 & |r_i| > 0.10\;\text{m} \;\text{(아웃라이어 절단)} \end{cases}
$$

**최종 관측 노이즈:**

$$
\sigma_{\text{obs},i}^2 = \frac{\sigma_\text{base}^2}{\max(w_\text{conf} \cdot w_\text{TLS},\; 0.01)}, \quad \sigma_\text{base} = 0.01\;\text{m}
$$

### 5.4 복셀 해시맵 (Voxel Hash Map)

공간을 **10cm** 복셀로 분할하여 `std::unordered_map<int64_t, VoxelCell>`에 저장.

**해시 함수:**

$$
h(x, y, z) = \lfloor x / s \rfloor + \lfloor y / s \rfloor \cdot 10^4 + \lfloor z / s \rfloor \cdot 10^8
$$

| 파라미터 | 값 |
|---------|-----|
| 복셀 크기 $s$ | 0.1m (10cm) |
| 복셀 당 최대 점 | 20개 |
| 최대 복셀 수 | 500,000 |
| KNN 검색 범위 | 27개 인접 복셀 ($3^3$) |
| LRU 제거 비율 | 초과 시 하위 10% 제거 |

### 5.5 키프레임 선택

새 키프레임 조건 (OR):

$$
\|\mathbf{p}_k - \mathbf{p}_{k-1}\| \geq 0.05\;\text{m} \quad \text{OR} \quad \arccos\left(\frac{\text{tr}(\Delta\mathbf{R}) - 1}{2}\right) \geq 2°
$$

### 5.6 발산 방지 (Divergence Guard)

ESKF 위치가 ARKit 사전값(prior)에서 1m 이상 벗어나면 ARKit 포즈로 리셋:

$$
\text{if}\; \|\mathbf{p}_\text{ESKF} - \mathbf{p}_\text{ARKit}\| > 1.0\;\text{m}: \quad \mathbf{x} \leftarrow \mathbf{x}_\text{ARKit}
$$

---

## 6. 포인트클라우드 전처리: Bundle & Discard

SLAM 엔진에 입력되기 전, 원시 포인트클라우드는 **Quantize → Gate → Optimize** 파이프라인을 거친다.

### 6.1 단계별 처리

```
원시 포인트 (~3,000/프레임)
    ↓
[1] Hard Confidence Gate: confidence == 0 제거
    ↓
[2] 복셀 해싱: 10cm 복셀로 분류
    ↓
[3] Density Gate: 점 < min_density(5개) 복셀 제거
    ↓
[4] Avg Confidence Gate: 복셀 평균 confidence < 1.5 제거
    ↓
[5] Centroid Emission: 복셀당 1개 평균점 (위치 + RGB 평균)
    ↓
번들링된 포인트 (~200-500/프레임)
```

이 전처리는 **80-95%** 의 점 감소를 달성하면서, 공간적으로 유의미한 점만 보존한다.

---

## 7. 포스트프로세싱 파이프라인

`optimizeAndExport()` 에서 4단계의 후처리를 수행한다.

### 7.1 Phase 1: SLAM 맵 로드

C++ 엔진의 `getFullMap()`에서 최적화된 전체 맵을 가져온다. 만약 SLAM 맵이 GPU 버퍼의 10% 미만이면 GPU 버퍼(ARKit 기반)를 유지한다.

### 7.2 Phase 2: Surface Thinning (표면 정제)

드리프트로 인한 **이중 벽(double wall)** 문제를 해결한다.

1. 점을 **50mm 셀**로 그룹화
2. 각 셀에서 최대 분산 축을 따라 **5mm 히스토그램** 생성
3. **15mm** 이상 갭 감지 → 중복 표면 레이어 판별
4. 슬라이딩 윈도우로 밀도 최대 **25mm** 표면만 보존
5. 안전 제한: 최대 40% 점 제거

### 7.3 Phase 3: Voxel Downsampling

| 알고리즘 | 복셀 크기 |
|---------|----------|
| DV-SLAM | 12mm (정밀) |
| ARKit | 20mm (조밀) |

복셀 내 위치·색상 평균, 최대 confidence 보존.

### 7.4 Phase 4: Statistical Outlier Removal (SOR)

1. **밀도 기반 제거:** 30mm 셀, 27-이웃 검사 → 이웃 < 3개인 점 제거
2. **IQR 기반 제거:** 센트로이드 거리의 $Q_3 + 3.0 \times \text{IQR}$ 초과 점 제거
3. 안전 제한: 최대 30% 점 제거

---

## 8. Apple ARKit 대비 DV-SLAM 이점

### 8.1 아키텍처 비교

```
┌─────────────────────────────────────────────────────┐
│                    Apple ARKit                       │
│  Visual-Inertial Odometry (VIO)                     │
│  카메라 이미지 기반 특징점 매칭 + IMU                    │
│  내부 구현 비공개 (블랙박스)                            │
│  LiDAR: depth anchor 보조 역할만                     │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│                    DV-SLAM                           │
│  LiDAR-Inertial Odometry (LIO)                      │
│  LiDAR 기하학 기반 + IMU + ARKit 사전값               │
│  ESKF + Point-to-Plane ICP                          │
│  LiDAR가 주 센서, ARKit은 안전망 역할                  │
└─────────────────────────────────────────────────────┘
```

### 8.2 정량적 이점

| 항목 | Apple ARKit | DV-SLAM |
|------|-------------|---------|
| **LiDAR 활용** | 깊이 보조 (메쉬/앵커용) | **주 센서** (기하학적 정합) |
| **드리프트 보정** | VIO 루프 클로저 (시각 의존) | Point-to-Plane ICP + ESKF |
| **조명 의존성** | 높음 (카메라 특징점 기반) | **낮음** (LiDAR는 능동 센서) |
| **포인트클라우드 품질** | 프레임별 독립 투영 | **글로벌 정합 최적화** |
| **노이즈 제거** | 기본 confidence 필터만 | Bundle & Discard + SOR + TLS |
| **이중 벽 보정** | 없음 | **Surface Thinning** |
| **바이어스 추정** | 내부 (제어 불가) | 온라인 ESKF ($\mathbf{b}_g$, $\mathbf{b}_a$) |
| **내보내기 복셀** | 20mm | **12mm** (더 정밀) |
| **커스터마이징** | 불가 | confidence 임계값, 거리 제한, 복셀 크기 조절 가능 |

### 8.3 DV-SLAM이 우수한 시나리오

**1. 저조도/무조명 환경**
- ARKit의 VIO는 카메라 특징점에 의존하여 어두운 곳에서 트래킹 품질 저하
- DV-SLAM의 LiDAR는 능동 센서로 조명과 무관하게 동작

**2. 텍스처 없는 평면 (흰 벽, 바닥)**
- ARKit은 시각적 특징점이 부족하면 드리프트 증가
- DV-SLAM은 기하학적 평면 자체를 관측하므로 영향 없음

**3. 반복적 구조 (복도, 창고)**
- ARKit VIO는 유사한 시각 패턴에서 혼란 가능
- DV-SLAM은 3D 기하학으로 구분하여 더 강건

**4. 고품질 내보내기 요구**
- ARKit은 프레임별 독립 투영 → 중복/불일치 가능
- DV-SLAM은 글로벌 맵 정합 + 4단계 후처리 → 깨끗한 출력

### 8.4 ARKit이 여전히 유리한 경우

- **실시간 렌더링:** ARKit GPU 파이프라인이 더 빠름 (DV-SLAM은 export-only)
- **루프 클로저:** ARKit의 시각 기반 장소 인식이 대규모 환경에서 유리
- **안정성:** ARKit은 Apple 하드웨어에 최적화된 성숙한 시스템

### 8.5 DV-SLAM의 하이브리드 전략

DV-SLAM은 ARKit을 **대체**하지 않고 **보완**한다:

$$
\mathbf{T}_\text{corrected} = \mathbf{T}_\text{correction} \cdot \mathbf{T}_\text{ARKit}
$$

$$
\mathbf{T}_\text{correction} = \mathbf{T}_\text{LIO} \cdot \mathbf{T}_\text{ARKit}^{-1}
$$

- 실시간 렌더링: ARKit 포즈 사용 (안정성)
- 내보내기: SLAM 최적화 맵 사용 (정밀도)
- 발산 시: ARKit으로 자동 폴백 (안전망)

이 하이브리드 접근법은 ARKit의 안정성과 DV-SLAM의 정밀도를 동시에 확보한다.

---

## 부록: 참고 알고리즘

DV-SLAM은 다음 오픈소스 알고리즘의 수학적 기반을 참고하여 iOS에 최적화하였다:

| 알고리즘 | 원본 | 핵심 차용 |
|---------|------|----------|
| FAST-LIO2 | HKU (Xu et al.) | ikd-Tree, IKFoM manifold ESKF |
| FAST-LIVO2 | HKU (Zheng et al.) | Voxel octree, 패치 기반 시각 추적 |
| DLIO | UCLA VECTR (Chen et al.) | NanoGICP, 기하학적 관측기 |
| Super-LIO | ECUST (Wang et al.) | OctVoxMap, SMW 업데이트 |

DV-SLAM은 이들의 **ESKF + Point-to-Plane ICP** 핵심을 유지하면서, ARKit confidence 신뢰도 활용, Bundle & Discard 전처리, Surface Thinning 후처리 등 **iOS LiDAR 특화 최적화**를 추가하였다.
