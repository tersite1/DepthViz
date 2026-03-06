# DV-SLAM 논문 수정 사항

> **대상**: IEEE RA-L 투고용 논문
> **원칙**:
> - "ARKit"은 구현 디테일 — contribution으로 내세우지 않음
> - **"mobile on-device"는 강점** — 내장 dToF + IMU + camera로 폰 위에서 real-time 동작
> - 논문은 **문제(sparse dToF regime)**와 **시스템(lightweight on-device LIO)**에 집중
> - **Confidence weighting은 contribution이 아님** — 센서 하드웨어가 뱉는 값을 가져다 쓴 것일 뿐. Ablation의 한 항목으로만 보고

---

## 1. 프레이밍 수정

### 변경 전 (문제점)
- "Confidence-aware observation model" → 핵심 contribution으로 포장
- "ARKit VIO pose 블랙박스 제거" → contribution처럼 서술
- Confidence가 핵심이면 공개 데이터셋 평가 불가 (confidence 있는 데이터셋 없음)

### 변경 후
- **핵심**: "sparse dToF regime (<500 pts/frame)에서 최초의 on-device LIO"
- ARKit: Experimental Setup에 1회만 언급
- Confidence: ablation table 한 줄 ("modest improvement")
- **ScanNet++ (ICCV 2023) 공개 데이터셋으로 평가 가능** — confidence 불필요

### 플랫폼 언급 방법
- **"iPhone"**: 적극 사용 — on-device real-time 동작은 강점
- **"내장 센서"**: 강조 — 외장 LiDAR 부착(Android+Mid360 논문)과 차별화
- **"ARKit"**: 최소화 — API 이름일 뿐

> **예시 문장들**:
> - "All processing runs on-device on an iPhone 15 Pro at 30Hz using only its built-in sensors."
> - "Unlike recent mobile mapping systems that attach external spinning LiDAR [ref], our method operates entirely with the smartphone's native dToF sensor."
> - "The entire pipeline — IMU propagation, visual tracking, and point-to-plane ICP — executes in real-time on the mobile device without offloading."

---

## 2. Abstract 수정

### 수정 방향
- 첫 문장: sparse dToF regime 문제 정의 (<500 pts/frame)
- 핵심 주장: lightweight LIO architecture가 이 regime에서 동작 가능하게 함
- 결과: ScanNet++ 평가, point-count sweep에서 collapse boundary 제시
- 플랫폼 언급: "on an iPhone at 30Hz" — 한 문장으로

---

## 3. Contributions 수정

### 변경 전
1. ~~Confidence-aware ICP for sparse dToF~~ ← **제거 (trivial)**
2. Sparse-regime characterization
3. Real-time multi-sensor mobile LIO

### 변경 후
1. **Sparse dToF regime에서 최초의 on-device LIO**: 기존 LIO가 10K+ pts를 전제하는 것과 달리, ~300-500 pts/frame에서 동작하는 LIO를 최초로 실증. ARKit VIO에 의존하지 않는 독립적 odometry
2. **Sparse-regime characterization**: point-count sweep 실험으로 LIO collapse boundary 최초 식별. 기존 LIO 논문이 다루지 않은 regime의 체계적 분석
3. **Lightweight LIO architecture for mobile SoC**: FAST-LIO2의 데이터 구조를 모바일 제약에 맞게 재설계:
   - ikd-tree → O(1) insert voxel hash map + LRU eviction (500K cap)
   - 10K-100K pts → max 100 obs stride sampling (LDLT 125x 연산 절감)
   - PCL/ROS/Boost/Sophus → Eigen-only self-contained (외부 의존성 zero)
   - Hot path heap 할당 제거: 고정 배열 VoxelCell[20], KNN[5], ring buffer
   - Depth + visual residuals를 단일 ESKF update에 스택 (행렬 연산 1회)
   - Bundle & Discard로 ICP 전 80% 점 절감
   - 결과: iPhone 15 Pro Max (A17 Pro)에서 30Hz real-time

> Confidence weighting (`R_obs = σ²/w(c_i)`)은 시스템에 포함되어 있으나, 센서 하드웨어가 제공하는 값을 그대로 사용한 것이므로 contribution이 아닌 ablation 항목으로 보고. "센서가 주는 걸 가져다 쓴 것"은 노벨티가 아님.

---

## 4. Section별 수정

### Section I. Introduction
- [ ] 첫 문단: dense LiDAR LIO의 성공 (FAST-LIO2, DLIO 등) → "하지만 sparse dToF에서는?"
- [ ] 두번째 문단: dToF 센서 특성 — 10K→500pts, 모바일 디바이스에서의 제약
- [ ] 세번째 문단: 기존 방법을 sparse regime에 적용하면 실패하는 이유
- [ ] Contribution bullet 3개 나열
- [ ] ~~ARKit~~ → 언급하지 않음
- [ ] ~~Confidence-aware~~ → contribution에서 제외

### Section III. System Overview
- [ ] Figure 1: 3-sensor pipeline diagram (dToF + Camera + IMU → ESKF)
- [ ] 플랫폼 무관하게 서술 — "dToF sensor provides per-pixel depth"
- [ ] ~~"ARKit serves as HAL"~~ → 삭제

### Section III-A. IMU Propagation
- [ ] ESKF predict (midpoint integration) 수식
- [ ] Static initialization: gravity alignment (30 samples)
- [ ] 좌표계: y-up convention

### Section III-B. Sparse dToF Point-to-Plane ICP
- [ ] Bundle & Discard preprocessing (voxel 30mm, density gate)
- [ ] Point-to-plane ICP with TLS robust kernel
- [ ] ~~Failure modes F1–F3~~ → 삭제 (confidence 중심 분석 불필요)
- [ ] ~~Proposition 1–2~~ → 삭제
- [ ] Sparse regime에서 ICP가 어려운 이유 (few correspondences, degeneracy) 간략 설명

### Section III-C. Visual Feature Tracking
- [ ] Shi-Tomasi + pyramidal KLT (standard — 간략히)
- [ ] Feature 3D initialization via dToF depth (stereo 불필요)
- [ ] Reprojection residual을 같은 ESKF update에 포함

### Section III-D. Tight Coupling in ESKF
- [ ] 단일 observation function: depth ICP residuals + visual reprojection residuals
- [ ] $H$, $z$, $R_{\text{obs}}$ 행렬 구조 (수식)
- [ ] Iterated Kalman with $P_{\text{pred}}$ 고정 (W-1 fix)

### Section III-E. Robustness
- [ ] Map seeding (5 keyframes before ICP)
- [ ] Correction clamping (rotation <5°, position <10cm)
- [ ] Non-convergence rejection
- [ ] Velocity clamp

### Section IV. Sparse Regime Analysis
- [ ] Point-count sweep 실험 설계 및 결과
- [ ] Collapse boundary 정의 및 식별
- [ ] ~~dToF noise characterization~~ → 선택사항 (자체 수집에서만 가능)

### Section V. Experimental Setup
- [ ] **주력 데이터셋: ScanNet++ (ICCV 2023)** — 460 scenes, iPhone 13 Pro, FARO GT
  - Depth (256×192, 16bit mm) + IMU (CMDeviceMotion) + RGB + Camera intrinsics + FARO GT mesh
  - 내 시스템과 동일한 CMDeviceMotion 포맷 → 호환
- [ ] 보조 데이터셋: 자체 수집 S1–S5 (closed-loop, confidence ablation용)
- [ ] 센서 스펙 표: iPhone 15 Pro Max — dToF (256×192, 30Hz), camera (1920×1440, 30Hz), IMU (100Hz), A17 Pro SoC
- [ ] Baselines: Naive ICP (uniform weight, no IMU), IMU-only, −Camera, KISS-ICP (sparse input)
- [ ] Metrics: ATE RMSE, loop closure drift %, map accuracy/completeness, per-frame latency

### Section VI. Results
- [ ] **Table: ScanNet++ trajectory/map 정확도** (DV-SLAM vs baselines)
- [ ] **Figure: point-count sweep** (핵심 figure) — ATE vs #points
- [ ] Figure: ablation bar chart (Full / −Camera / −TLS / −B&D / −Confidence / IMU-only)
- [ ] Runtime table: per-frame latency on iPhone 15 Pro Max
- [ ] Qualitative: 재구성 결과 비교 이미지
- [ ] "All results are obtained from on-device processing without GPU offloading or post-processing"

### Section VII. Discussion
- [ ] Sparse regime에서 tight coupling의 중요성
- [ ] Confidence는 modest improvement — ablation 결과 인용
- [ ] 한계: loop closure 없음, dense LiDAR 대비 정확도
- [ ] Future work: loop closure, learned depth completion

---

## 5. Figure 목록

| # | 내용 | 우선순위 |
|---|---|---|
| Fig.1 | System overview (3-sensor pipeline → ESKF) | 필수 |
| Fig.2 | **Point-count sweep: ATE vs #points** | **최핵심** |
| Fig.3 | Ablation bar chart | 필수 |
| Fig.4 | Trajectory 비교 (overhead view, ScanNet++ scenes) | 필수 |
| Fig.5 | Map quality: 재구성 vs FARO GT 비교 | 필수 |
| Fig.6 | Qualitative reconstruction 비교 | 선택 |

---

## 6. 리뷰어 대응 준비

| 예상 질문 | 대응 |
|---|---|
| "FAST-LIVO2랑 뭐가 다르나" | 20x sparser regime, on-device mobile, collapse boundary 최초 분석 |
| "성능이 왜 dense LIO보다 나쁘나" | 20x fewer points — 동일 정확도 불가능. contribution은 "가능하게 만든 것" |
| "500pts로 LIO 가능한 건 알려진 거 아닌가" | 기존 논문 중 500pts 이하 체계적 테스트 = 없음 |
| "왜 loop closure 안 하나" | Odometry 정확도가 우선. LC는 future work. 제목도 LIO |
| "Engineering contribution 아닌가" | KISS-ICP도 새 알고리즘 없이 RA-L accept. 새로운 regime + 실증 = valid |
| "왜 공개 데이터셋이 1개뿐인가" | ScanNet++이 유일하게 dToF depth + CMDeviceMotion IMU + FARO GT 동시 제공 |
| "Confidence는 왜 안 쓰나" | 센서 제공 값을 가져다 쓴 것은 contribution이 아님. Ablation에서 modest improvement 보고 |

---

## 7. 실험 설계 (Experiment Design)

> **원칙**: 각 실험은 최소 1개 contribution을 직접 뒷받침해야 한다.

---

### 7.1 데이터

**주력: ScanNet++ (공개)**
- 460 scenes 중 다양한 환경 10–20개 선별
- Depth (256×192) + IMU (CMDeviceMotion) + RGB + FARO GT mesh
- 오프라인 replay: depth/IMU/RGB를 시간순 재생 → LIO 파이프라인에 입력

**보조: 자체 수집**

| ID | 환경 | 거리 | 목적 |
|---|---|---|---|
| S1 | 일반 실내 | ~20m | Ablation 기준 + confidence ablation |
| S2 | 긴 복도 | ~30m | 직선 drift |
| S3 | 빠른 회전 | ~15m | IMU+ICP robustness |
| S4 | 유리/반사면 | ~15m | 어려운 환경 |
| S5 | 야외 | ~20m | 환경 다양성 |

자체 수집: closed-loop (시작=끝) + COLMAP pseudo-GT

---

### 7.2 Experiment 1: Point-Count Sweep ⭐ (핵심)

> **뒷받침**: C1 (sparse regime에서 동작), C2 (collapse boundary)

**목적**: point 수를 줄여가며 LIO가 붕괴하는 경계 식별

**방법**:
1. ScanNet++ scene 3–5개 선별
2. Stride 변수: {1, 2, 4, 8, 12, 16} → 예상 point 수: {~3000, ~1500, ~500, ~250, ~170, ~100}
3. 각 stride에서 LIO 실행, FARO GT 대비 ATE RMSE 측정
4. 총 18–30 runs (6 strides × 3–5 scenes)

**산출물**:
- **Figure 2** (논문의 핵심 figure):
  - X축: #points per frame (log scale)
  - Y축: ATE RMSE [m] (log scale)
  - Collapse boundary 수직선 표시

**핵심 메시지**: "LIO remains viable down to ~N pts/frame; below this threshold it collapses"

---

### 7.3 Experiment 2: Ablation Study

> **뒷받침**: C1 (각 구성요소 기여), C3 (tight coupling 효과)

**방법**: ScanNet++ 기준 scene + 자체 수집 S1

| Config | 설명 | 비교 목적 |
|---|---|---|
| **Full** | Camera + dToF + IMU + TLS + B&D | 기준 |
| −Camera | dToF + IMU only | Visual tracking 기여 |
| −TLS | Robust kernel 제거 (L2 only) | Outlier rejection 효과 |
| −B&D | Bundle&Discard 없음 | Preprocessing 효과 |
| −Confidence | w_i = 1.0 (uniform weight) | Confidence 기여 (자체수집만) |
| IMU-only | ICP/visual 없음 | Lower bound (dead reckoning) |

**산출물**: ATE RMSE table + bar chart

**예상 순서**: Full < −TLS < −B&D < −Camera ≪ IMU-only
(−Confidence는 Full보다 약간 나쁜 정도 예상 — "modest improvement")

---

### 7.4 Experiment 3: Trajectory 정확도 (ScanNet++)

> **뒷받침**: C1 (시스템 전체 성능)

**방법**: ScanNet++ 10–20 scenes

| Method | 설명 |
|--------|------|
| **DV-SLAM (ours)** | Full pipeline |
| Naive ICP (no IMU) | ICP only, uniform weight |
| IMU-only | Dead reckoning |
| KISS-ICP | 기존 LiDAR odometry, sparse input |

**산출물**: Table (scene별 ATE RMSE) + trajectory overlay figure

---

### 7.5 Experiment 4: Map Quality (ScanNet++)

> **뒷받침**: 보조 evidence

**방법**: LIO로 생성한 point cloud vs FARO GT mesh
- CloudCompare로 cloud-to-mesh distance

**산출물**: Accuracy [cm], Completeness [cm], F-score @ 5cm

---

### 7.6 Experiment 5: Runtime

> **뒷받침**: C3 (on-device real-time)

**산출물**:

| Stage | Mean [ms] | Std [ms] | Max [ms] |
|---|---|---|---|
| IMU predict | | | |
| Bundle & Discard | | | |
| Visual tracking (KLT) | | | |
| ICP (ESKF update) | | | |
| **Total per frame** | | | |
| Peak memory [MB] | | | |

---

### 7.7 실험-Contribution 매핑

| 실험 | C1 (Sparse LIO) | C2 (Regime analysis) | C3 (Mobile arch) |
|---|---|---|---|
| E1: Point-count sweep | ✅ | ✅ **핵심** | |
| E2: Ablation | ✅ | | ✅ |
| E3: Trajectory (ScanNet++) | ✅ **핵심** | | ✅ |
| E4: Map quality | 보조 | | 보조 |
| E5: Runtime | | | ✅ **핵심** |

**최소 필수**: E1 + E2 + E3 + E5
**E4는 있으면 좋음** (FARO GT 활용)

---

### 7.8 실행 우선순위

```
1순위: ScanNet++ 데이터 다운로드 + 오프라인 replay 파이프라인 구축
2순위: E3 (Trajectory) ← ScanNet++ 1개 scene으로 시작, FARO GT 대비 확인
3순위: E1 (Point sweep) ← 같은 scene에서 stride 변경
4순위: E2 (Ablation) ← config 변경하며 반복
5순위: E5 (Runtime) ← profiling 코드 이미 있음
6순위: E4 (Map quality) ← CloudCompare
7순위: 자체 수집 S1–S5 ← confidence ablation 등 보조 실험
```

> **E1 + E3가 논문의 핵심 결과**. ScanNet++에서 돌아가는 것이 확인되면 논문 골격 완성.

---

## 8. 핵심 사실

- **테스트 디바이스**: iPhone 15 Pro Max (A17 Pro SoC)
- **빌드/테스트 완료**: on-device real-time 동작 확인, App Store 배포 중
- **주력 평가 데이터셋**: ScanNet++ (ICCV 2023, 460 scenes, FARO GT)
- **IMU 호환**: ScanNet++의 CMDeviceMotion = 내 앱의 CMDeviceMotion (동일 포맷)

---

## 9. 체크리스트 (제출 전)

- [ ] ARKit 단어 검색 → Experimental Setup 외 전부 제거
- [ ] Confidence를 contribution으로 주장하는 문장 전부 제거
- [ ] 모든 contribution이 실험으로 뒷받침되는지 확인
- [ ] Point-count sweep figure 완성
- [ ] ScanNet++ 평가 결과 table 완성
- [ ] Ablation table 완성
- [ ] RA-L 6+2 페이지 제한 확인
- [ ] IEEE format compliance
- [ ] References: FAST-LIO2, KISS-ICP, GenZ-ICP, FAST-LIVO2, ScanNet++ 포함
