# DV-SLAM 논문 수정 사항

> **대상**: IEEE RA-L 투고용 논문
> **원칙**:
> - "ARKit"은 구현 디테일 — 제거/사용 여부를 contribution으로 내세우지 않음
> - **"mobile on-device"는 강점** — 내장 dToF + IMU + camera로 폰 위에서 real-time 동작. 외장 LiDAR 부착이 아님
> - 논문은 **문제(sparse dToF regime)**와 **해법(confidence-aware observation model)**에 집중하되, mobile real-time 동작을 시스템 기여로 명시

---

## 1. 프레이밍 수정 — ARKit 언급 최소화

### 변경 전 (문제점)
- "ARKit VIO pose 블랙박스 제거" → contribution처럼 서술
- "ARKit = sensor HAL" → 구현 디테일을 과하게 강조
- "ARKit 대비 X% 달성" → baseline이 비공개 시스템

### 변경 후
- ARKit 단어 자체를 논문에서 제거 (또는 Experimental Setup에 1회만)
- 플랫폼 무관하게 서술: "dToF sensor + IMU + monocular camera"
- Baseline은 공개 알고리즘: FAST-LIO2, Naive ICP (uniform weight)

### 플랫폼 언급 방법
- **"iPhone"**: 적극 사용 — 모바일 디바이스에서 on-device real-time 동작은 강점
- **"내장 센서"**: 강조 — 외장 LiDAR 부착(Android+Mid360 논문)과 차별화
- **"ARKit"**: 최소화 — API 이름일 뿐, contribution 아님

> **예시 문장들**:
> - "All processing runs on-device on an iPhone 15 Pro at 30Hz using only its built-in sensors."
> - "Unlike recent mobile mapping systems that attach external spinning LiDAR [ref], our method operates entirely with the smartphone's native dToF sensor."
> - "The entire pipeline — IMU propagation, visual tracking, and confidence-weighted ICP — executes in real-time on the mobile device without offloading."

---

## 2. Abstract 수정

### 현재 문제
- Multi-sensor tight coupling 서술 부족
- 플랫폼 의존적 언어

### 수정 방향
- 첫 문장: sparse dToF regime 문제 정의 (<500 pts/frame)
- 핵심 주장: confidence metadata가 LIO를 가능하게 하는 enabler
- 결과: point-count sweep에서 collapse boundary 제시, confidence ablation
- 플랫폼 언급 없음

---

## 3. Contributions 수정

### 현재 (todo_work.md B1)
1. Confidence-aware ICP for sparse dToF
2. Sparse-regime characterization
3. Real-time multi-sensor mobile LIO

### 수정 후
1. **Confidence-aware observation model**: dToF 센서의 per-point confidence를 ICP observation noise $R_{\text{obs}}$에 반영. Failure modes (F1–F3) 분석, uniform weight 대비 54% variance inflation 정량화 (Proposition 2)
2. **Sparse-regime characterization**: point-count sweep (50–3000 pts) × confidence ON/OFF 실험으로 LIO collapse boundary 최초 식별. 기존 LIO 논문(FAST-LIO2 등)이 다루지 않은 regime
3. **On-device mobile LIO system**: Camera + dToF + IMU tight coupling을 single ESKF update에서 수행. 외장 LiDAR 없이 스마트폰 내장 센서만으로 on-device real-time 동작 (30Hz, iPhone 15 Pro). 기존 모바일 매핑 시스템(외장 Mid360 부착)과 달리 추가 하드웨어 불필요

> Note: C3은 system contribution이지만, "외장 LiDAR 없이 폰 내장 센서만으로 real-time LIO"는 기존에 없는 시스템이므로 단순 적용 이상의 의미.

---

## 4. Section별 수정

### Section I. Introduction
- [ ] 첫 문단: dense LiDAR LIO의 성공 (FAST-LIO2, DLIO 등) → "하지만 sparse dToF에서는?"
- [ ] 두번째 문단: dToF 센서 특성 — 10K→500pts, confidence metadata 존재
- [ ] 세번째 문단: 기존 adaptive weighting (GenZ-ICP, D²-LIO)과의 차이 — geometric post-hoc vs sensor-provided
- [ ] Contribution bullet 3개 나열
- [ ] ~~ARKit 블랙박스~~ → 언급하지 않음

### Section III. System Overview
- [ ] Figure 1: 3-sensor pipeline diagram (dToF + Camera + IMU → ESKF)
- [ ] 플랫폼 무관하게 서술 — "dToF sensor provides depth + confidence per pixel"
- [ ] ~~"ARKit serves as HAL"~~ → 삭제

### Section III-A. IMU Propagation
- [ ] ESKF predict (midpoint integration) 수식
- [ ] Static initialization: gravity alignment (30 samples)
- [ ] 좌표계: y-up convention, $\mathbf{g} = [0, -9.81, 0]^T$

### Section III-B. Confidence-Aware dToF ICP (핵심 섹션)
- [ ] Bundle & Discard preprocessing (voxel 30mm, density/confidence gate)
- [ ] Point-to-plane ICP with confidence-weighted $R_{\text{obs}}$
- [ ] Failure modes F1 (low confidence noise), F2 (multipath), F3 (surface boundary)
- [ ] Proposition 1–2: variance inflation 유도
- [ ] TLS robust kernel

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
- [ ] ~~Divergence guard: ARKit 비교~~ → 공분산 + velocity 모니터링

### Section IV. Sparse Regime Analysis
- [ ] dToF noise characterization: σ(distance, confidence) 실측
- [ ] Point-count sweep 실험 설계
- [ ] Collapse boundary 정의 및 식별

### Section V. Experimental Setup
- [ ] 데이터셋: self-collected S1–S5 (표 형태)
- [ ] Ground truth: closed-loop drift, FARO/Livox reference (있으면)
- [ ] 센서 스펙 표: **iPhone 15 Pro Max** — dToF (256×192, 30Hz, 3-level confidence), camera (1920×1440, 30Hz), IMU (100Hz, 6-axis), A17 Pro SoC
- [ ] **On-device 실행 환경 명시**: A17 Pro SoC, 처리 시간 breakdown (IMU/VIO/ICP/total), peak memory
- [ ] Baselines: FAST-LIO2 (ROS bridge), Naive ICP (uniform weight), platform VIO (비공개 시스템이므로 "upper bound reference"로 위치)
- [ ] Metrics: ATE RMSE, loop closure drift %, map accuracy/completeness, **per-frame latency**

### Section VI. Results
- [ ] Table: trajectory 정확도 비교 (DV-SLAM vs baselines)
- [ ] Figure: point-count sweep (핵심 figure — 논문의 90%)
- [ ] Figure: ablation bar chart (Full / −Confidence / −Camera / −TLS / −B&D)
- [ ] **Runtime table**: per-frame latency on iPhone 15 Pro Max (A17 Pro) — IMU predict, visual tracking, ICP, total
- [ ] Qualitative: 재구성 결과 비교 이미지
- [ ] **On-device 강조**: "All results are obtained from on-device processing without GPU offloading or post-processing"

### Section VII. Discussion
- [ ] Sparse regime에서 confidence의 역할 분석
- [ ] 한계: loop closure 없음, dense LiDAR 대비 정확도
- [ ] ~~ARKit 블랙박스 제거의 의의~~ → "재현성 및 이식성" (플랫폼 무관 서술)
- [ ] Future work: loop closure, learned confidence refinement

---

## 5. Figure 목록

| # | 내용 | 우선순위 |
|---|---|---|
| Fig.1 | System overview (3-sensor pipeline → ESKF) | 필수 |
| Fig.2 | dToF noise characterization: σ vs distance × confidence | 필수 |
| Fig.3 | **Point-count sweep: ATE vs #points × confidence ON/OFF** | **최핵심** |
| Fig.4 | Ablation bar chart | 필수 |
| Fig.5 | Trajectory 비교 (overhead view) | 필수 |
| Fig.6 | Qualitative reconstruction 비교 | 선택 |

---

## 6. 리뷰어 대응 준비

| 예상 질문 | 대응 |
|---|---|
| "Confidence weight는 trivial" | F1–F3 failure analysis + 54% variance + ablation collapse |
| "GenZ-ICP도 adaptive weight" | Geometric post-hoc vs sensor-provided physical confidence (SPAD photon count) |
| "FAST-LIVO2랑 뭐가 다르나" | 20x sparser regime, confidence metadata, collapse boundary characterization |
| "성능이 왜 dense LIO보다 나쁘나" | 20x fewer points — 동일 정확도가 불가능. contribution은 "가능하게 만든 것" |
| "500pts로 LIO 가능한 건 알려진 거 아닌가" | 기존 논문 중 500pts 이하 체계적 테스트 = 없음. 가능성 자체를 보인 것이 contribution |
| "왜 loop closure 안 하나" | Odometry 정확도가 우선. LC는 future work. 제목도 LIO |

---

## 7. 실험 설계 (Experiment Design)

> **원칙**: 각 실험은 최소 1개 contribution을 직접 뒷받침해야 한다. "있으면 좋겠다" 수준의 실험은 제외.

---

### 7.1 데이터 수집 시퀀스

| ID | 환경 | 거리 | 목적 | 특이사항 |
|---|---|---|---|---|
| S1 | 일반 실내 (방+거실) | ~20m | 기본 성능 평가 | Ablation 기준 시퀀스 |
| S2 | 긴 복도 | ~30m | 직선 drift 평가 | Degeneracy-prone (단일 평면) |
| S3 | 빠른 회전 (>2 rad/s) | ~15m | IMU+ICP 결합 robustness | 모션 블러, 피쳐 트래킹 스트레스 |
| S4 | 유리/반사 표면 | ~15m | Confidence 효과 극대화 | F2 failure mode (multipath) |
| S5 | 밝은 야외 | ~20m | SPAD saturation, 환경 다양성 | dToF range 한계 테스트 |

**모든 시퀀스 공통**:
- Closed loop (시작점 = 종료점) → loop closure drift 측정 가능
- Raw 로깅: dToF depth+confidence (30Hz) + camera gray (30Hz) + IMU (100Hz)
- Platform VIO pose도 로깅 → upper bound reference 용도

**데이터 수집 프로토콜**:
1. 디바이스 수평으로 들고 3초 정지 (IMU static init)
2. 천천히 걷기 시작 (~0.3 m/s)
3. 시퀀스 경로 1바퀴 후 시작점으로 복귀
4. 3초 정지 후 종료

---

### 7.2 Experiment 1: dToF Noise Characterization

> **뒷받침**: C1 (Proposition 2: 54% variance inflation)

**목적**: dToF 센서의 depth noise σ를 distance × confidence별로 실측

**방법**:
1. 평평한 벽에 디바이스를 삼각대 고정
2. 5개 거리: {0.5, 1.0, 2.0, 3.0, 4.0} m
3. 각 거리에서 100 static frames 수집
4. 각 frame의 각 pixel에서: confidence level (0/1/2) 기록 + plane fitting residual 계산
5. σ(distance, confidence) 표 + 그래프 생성

**산출물**:
- Table: σ [mm] vs distance × confidence
- Figure 2: 3개 confidence curve (0, 1, 2)와 σ 관계
- Proposition 2 검증: confidence=0의 variance가 confidence=2 대비 실제로 ~1.54x인지

**예상 결과**:
- confidence=2: σ ≈ 5–15mm (거리 비례)
- confidence=1: σ ≈ 15–30mm
- confidence=0: σ ≈ 30–100mm+ (또는 완전 invalid)
- → "confidence=0 제거가 필수"를 수치로 증명

---

### 7.3 Experiment 2: Point-Count Sweep ⭐ (핵심 실험)

> **뒷받침**: C2 (Sparse-regime characterization, collapse boundary)

**목적**: point 수를 줄여가며 LIO가 붕괴하는 경계를 식별

**방법**:
1. S1 시퀀스를 기준으로 사용
2. Stride 변수: {1, 2, 4, 8, 12, 16} → 예상 point 수: {~3000, ~1500, ~500, ~250, ~170, ~100}
3. 각 stride에서 **2가지 조건**: confidence ON (w_i = f(c_i)) / confidence OFF (w_i = 1.0)
4. 총 12 runs (6 strides × 2 conditions)
5. 각 run의 ATE RMSE 측정 (closed-loop drift로 계산 가능)

**산출물**:
- **Figure 3** (논문의 핵심 figure):
  - X축: #points per frame (log scale)
  - Y축: ATE RMSE [m] (log scale)
  - 2개 curve: confidence ON (파랑), confidence OFF (빨강)
  - Collapse boundary 수직선 표시
- 예상 형태:
  ```
  ATE
  [m]
   1.0 ─ ─ ─ ─ ─ ─ ─ ─ ×─────── OFF (collapse)
                        /
   0.1 ─ ─ ─ ─ ─ × ─ ×
                /   /
   0.01 ●──●──●──●
        3K  1.5K 500 250 170 100
                  ↑
            collapse boundary
  ```
- ON curve는 ~200pts까지 유지, OFF curve는 ~500pts에서 발산 → **confidence가 2.5x 더 sparse까지 버틴다**

**핵심 메시지**: "Confidence weighting extends the operational regime of sparse dToF LIO by Nx"

---

### 7.4 Experiment 3: Ablation Study

> **뒷받침**: C1 (각 구성요소 기여 분리), C3 (tight coupling 효과)

**방법**: S1 시퀀스, 기본 stride=4 (~500pts)

| Config | 설명 | 비교 목적 |
|---|---|---|
| **Full** | Camera + dToF + IMU, confidence + B&D + TLS | 기준 |
| −Confidence | w_i = 1.0 (uniform weight) | C1 핵심: confidence 기여 격리 |
| −Camera | dToF + IMU only (visual 없음) | C3: camera 기여 격리 |
| −TLS | Robust kernel 제거 (L2 only) | Outlier rejection 효과 |
| −B&D | Bundle&Discard 없음 (raw points) | Preprocessing 효과 |
| IMU-only | IMU predict만, ICP/visual 없음 | Lower bound (dead reckoning drift) |

**산출물**:
- Table: ATE RMSE, loop drift %, 평균 ICP convergence iter, obs count
- Figure 4: bar chart (각 config의 ATE RMSE)

**예상 결과 순서** (좋은→나쁜):
```
Full < −TLS < −B&D < −Camera < −Confidence ≪ IMU-only
```
- **핵심**: −Confidence가 −Camera보다 나빠야 thesis 증명
- 즉 "confidence가 camera보다 더 중요하다" = 강력한 결론

---

### 7.5 Experiment 4: Trajectory 정확도 (Baseline 비교)

> **뒷받침**: C3 (시스템 전체 성능)

**방법**: 전 시퀀스 S1–S5

| Baseline | 설명 | 비교 의미 |
|---|---|---|
| **DV-SLAM (ours)** | Full pipeline | — |
| Naive ICP | Uniform weight, no confidence/TLS | Confidence 효과 격리 |
| Platform VIO | 디바이스 내장 VIO (비공개) | Upper bound reference |

**산출물**:
- Table: 시퀀스별 ATE RMSE [m], loop closure drift [%]
- Figure 5: overhead trajectory 비교 (S1, S2 기준)

**Notes**:
- FAST-LIO2 (ROS bridge)는 가능하면 추가하되, iPhone dToF를 ROS로 브릿징하는 공수가 큼 → optional
- Platform VIO는 "upper bound"로만 위치. "이기겠다"가 아니라 "합리적 거리 내에 있다"를 보임
- **S2 (복도)**와 **S4 (유리)**에서 DV-SLAM이 Naive ICP를 크게 이기면 강력한 evidence

---

### 7.6 Experiment 5: Runtime 분석

> **뒷받침**: C3 (on-device real-time 가능성)

**방법**: S1 시퀀스, 프레임별 처리 시간 로깅 (이미 profiling 코드 있음)

**산출물**:
- Table:

| Stage | Mean [ms] | Std [ms] | Max [ms] |
|---|---|---|---|
| IMU predict (per sample) | | | |
| Bundle & Discard | | | |
| Visual tracking (KLT) | | | |
| ICP (ESKF update) | | | |
| **Total per frame** | | | |
| Peak memory [MB] | | | |

- **핵심 수치**: total < 33ms (30Hz) 충족 여부
- 비교: Android+Mid360 논문의 Faster-LIO latency와 비교 가능

---

### 7.7 Experiment 6: Map Quality (Optional — GT 있을 때)

> **뒷받침**: 보조 evidence

**방법**: GT mesh/point cloud 있는 경우만 수행
- FARO S70 스캔 또는 Livox Mid-70 reference map
- CloudCompare로 cloud-to-cloud distance 계산

**산출물**:
- Accuracy [cm]: mean distance from ours → GT
- Completeness [cm]: mean distance from GT → ours
- F-score @ 5cm threshold

**현실적 판단**: FARO/Livox가 없으면 이 실험은 패스. Closed-loop drift만으로 충분.

---

### 7.8 실험-Contribution 매핑

| 실험 | C1 (Confidence) | C2 (Sparse regime) | C3 (Mobile system) |
|---|---|---|---|
| E1: Noise characterization | ✅ 직접 | | |
| E2: Point-count sweep | ✅ ON/OFF 비교 | ✅ **핵심** | |
| E3: Ablation | ✅ **핵심** | | ✅ −Camera |
| E4: Trajectory accuracy | | | ✅ **핵심** |
| E5: Runtime | | | ✅ 직접 |
| E6: Map quality | | | 보조 |

**최소 필수 실험**: E1 + E2 + E3 + E4 + E5 (5개)
**E6은 GT 있으면** 추가

---

### 7.9 실험 우선순위 (실행 순서)

```
1순위: E5 (Runtime)      ← 코드에 이미 profiling 있음, 바로 측정 가능
2순위: E4 (Trajectory)    ← S1 하나로 시작, closed-loop drift 확인
3순위: E3 (Ablation)      ← S1에서 config 변경하며 반복
4순위: E2 (Point sweep)   ← S1에서 stride 변경하며 반복 ⭐
5순위: E1 (Noise)         ← 삼각대 + 벽 필요, 별도 수집
6순위: E6 (Map quality)   ← GT 확보 후
```

> **E2 + E3가 논문의 핵심 결과**. 나머지는 뒷받침.
> E4의 S2–S5는 E2/E3 결과 확인 후 추가 수집해도 됨.

---

## 8. 핵심 사실

- **테스트 디바이스**: iPhone 15 Pro Max (A17 Pro SoC)
- **빌드/테스트 완료**: on-device real-time 동작 확인
- 논문 서술: "Implemented and validated on iPhone 15 Pro Max. The entire pipeline runs on-device in real-time at 30Hz without external computation."

---

## 8. 체크리스트 (제출 전)

- [ ] ARKit 단어 검색 → Experimental Setup 외 전부 제거
- [ ] 모든 contribution이 실험으로 뒷받침되는지 확인
- [ ] Point-count sweep figure 완성
- [ ] Ablation table 완성
- [ ] RA-L 6+2 페이지 제한 확인
- [ ] IEEE format compliance
- [ ] References: FAST-LIO2, GenZ-ICP, D²-LIO, FAST-LIVO2, ARKitScenes 포함
- [ ] 코드-논문 수식 일치 최종 검증
