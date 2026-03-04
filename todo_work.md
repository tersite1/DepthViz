# DV-SLAM TODO Work

> **핵심 원칙**: 논문과 프로덕트는 별개 트랙.
> 공통: ARKit = 센서 HAL (camera, depth, confidence, IMU, intrinsics). ARKit VIO pose(블랙박스) 제거.
> 논문 = Camera + LiDAR + IMU tight coupling, 자체 pose estimation
> 프로덕트 = 최대한 잘 작동 (ARKit pose 활용 가능)

---

# Track A: 프로덕트 (App Store 제출)

## A1. 현재 상태
- [x] 표면 씬닝: gap-only 복원 (0.1% 제거)
- [x] 복셀: 8mm 복원
- [x] ICP iterations: 5
- [x] Divergence guard: 회전 30° + velocity reset
- [x] FileView Z-axis slider
- [x] 빌드 v2.2 Build 15

## A2. 남은 프로덕트 작업
- [ ] 실기기 테스트 — 현재 설정으로 스캔 품질 확인
- [ ] 모니터 겹침 (ghost) 수준 평가
- [ ] IAP — App Store Connect에 등록
- [ ] App Store 심사 제출

## A3. 프로덕트 아키텍처 (현재 유지)
```
ARKit → depth + confidence + IMU + ARKit VIO pose
DepthVizEngine: prior_pose = arkit_pose
  ├── non-keyframe → ARKit pose 사용
  └── keyframe → LIO(ARKit prior, points) → refined pose
```
- 프로덕트는 ARKit VIO 활용 → 안정적, 건드리지 않음
- 논문 완성 후 자체 pose estimation이 충분히 좋으면 프로덕트도 전환 가능

---

# Track B: 논문 (IEEE RA-L 제출)

## B1. 논문 프레이밍 (확정)

### 핵심 Thesis
> "sparse dToF regime (<500 pts/frame)에서 confidence metadata가 LIO를 가능하게 하는 핵심 enabler"

### 포지셔닝
- **Camera + LiDAR + IMU tight coupling** on mobile dToF
- ARKit = 센서 HAL (raw data만 사용). **ARKit VIO pose 사용 안함 (블랙박스 제거)**
- 자체 pose estimation: IMU propagation + Visual feature tracking + Confidence-weighted dToF ICP
- 모든 알고리즘이 서술 가능, 재현 가능

### Contributions
1. **Confidence-aware ICP for sparse dToF** — F1-F3 failure modes, Propositions 1-2 (54% variance inflation)
2. **Sparse-regime characterization** — point-count sweep, collapse boundary 식별
3. **Real-time multi-sensor mobile LIO** — Camera+LiDAR+IMU tight coupling, Eigen+OpenCV, 자체 pose estimation

### 논문 서술 방향
> "DV-SLAM performs full 6-DoF pose estimation by tightly coupling IMU propagation, sparse visual feature tracking, and confidence-weighted point-to-plane ICP on dToF depth. The platform (ARKit) serves solely as a hardware abstraction layer for sensor data acquisition — no platform-computed pose is used."

---

## B2. 논문용 코드 수정

### 코드 분기 전략
```
main branch   → 프로덕트 (ARKit pose 활용)
paper branch  → 논문용 (자체 pose estimation, 블랙박스 제거)
```

### 수정 개요: 2단계

**Step 1: ARKit VIO 제거 + IMU-only LIO (기존 계획)**
**Step 2: Visual feature tracking 추가 (Camera 활용)**

### Step 1: ARKit VIO 제거 (IMU + depth only)

#### A. 좌표계 통일: y-up
```
DV_Types.h: g = V3d(0, -9.81, 0)  // y-up: gravity = -y (ARKit convention)
정적 초기화: body gravity를 world -y에 정렬
출력: y-up 그대로 → 렌더링 변환 불필요
```

#### B. IMU 정적 초기화 (`DV_ESKF.cpp`)
```
1. ~30 IMU 샘플 수집 (100Hz × 0.3s)
2. g_body = mean(acc_samples)
3. R_0 = rotation aligning g_body to (0, -9.81, 0)
4. state_.R = R_0, state_.p = 0, state_.v = 0
5. state_.g = (0, -9.81, 0)
```

#### C. `DV_LIOBackend.cpp` — prior_pose 제거
```
변경 전: Matrix4d process(const Matrix4d& prior_pose, const vector<PointXYZIC>& points)
변경 후: Matrix4d process(const vector<PointXYZIC>& points)
  → 내부 ESKF 상태가 prior
```

#### D. Divergence Guard — 공분산 기반
```
변경 전: ESKF↔ARKit 비교 → ARKit으로 리셋
변경 후: 공분산 trace + 속도 크기 + ICP 잔차 모니터링
  → velocity damping, 바이어스 감쇠
```

#### E. `DepthVizEngine.cpp` — ESKF 상태로 대체
```
non-keyframe → last_optimized_pose_ = eskf_state.poseMatrix()
keyframe → refined = lio_->process(points)
```

#### F. `SLAMService.mm` — pushARKitPose() 제거

### Step 2: Visual Feature Tracking 추가 (Camera 활용)

#### G. Feature 추출 & 추적
```
구현: OpenCV 활용
  - cv::goodFeaturesToTrack() → Shi-Tomasi corners (~100-200 features)
  - cv::calcOpticalFlowPyrLK() → KLT optical flow tracking
  - RANSAC outlier rejection

입력: ARKit camera frame (이미 컬러 매핑용으로 받고 있음)
출력: 2D feature correspondences (prev_frame ↔ curr_frame)
```

#### H. Visual Residual을 ESKF에 추가
```
현재 ESKF update:
  residual_depth = n^T(R*p_cam + t - centroid)     ← point-to-plane (depth)

추가:
  residual_visual = project(R*P_3d + t) - uv_obs   ← reprojection (camera)

두 residual을 하나의 H, z 행렬에 쌓아서 같은 updateObserve()에 전달
```

#### I. Feature 3D 초기화
```
새 feature 감지 시:
  - 해당 pixel의 depth map에서 깊이 읽기 (dToF)
  - confidence 체크 → low confidence면 초기화 안 함
  - p_3d = K^{-1} * [u, v, 1] * depth → camera frame
  - world frame으로 변환: P_world = R * p_3d + t

dToF depth가 있으므로 stereo/triangulation 불필요 → 구현 간단
```

#### J. Feature 관리
```
- 최대 200 features 유지
- tracking 실패 feature 제거
- 3프레임 이상 추적된 feature만 관측에 사용
- 매 keyframe마다 새 feature 보충
```

### ESKF Observation 구조 (최종)
```
한 keyframe에서의 관측:
┌─────────────────────────────────────┐
│ Depth observations (~200 pts)       │ ← confidence-weighted point-to-plane
│   residual: n^T(Rp + t - c)        │    R_obs: σ(conf) 기반
│   H: [-n^T R [p]_x | n^T | 0...]  │
├─────────────────────────────────────┤
│ Visual observations (~100 features) │ ← reprojection error
│   residual: π(RP + t) - uv         │    R_obs: ~1 pixel
│   H: [∂π/∂θ | ∂π/∂t | 0...]       │
└─────────────────────────────────────┘

총 관측: ~300개 → 18D 상태 추정에 충분
```

### 타이밍/주파수 설계
```
IMU 100Hz:    ─●─●─●─●─●─●─●─●─●─●─●─●─●─●─●─
Camera 30Hz:  ────────C────────C────────C────────  (feature tracking)
Depth 30Hz:   ────────D────────D────────D────────  (동기)
Keyframe:     ─────────────────K─────────────────

● = ESKF predict (IMU midpoint, ~0.01ms)
C = feature tracking (KLT, ~2ms)
D = depth frame 도착
K = keyframe → Bundle&Discard + feature tracking → ESKF update (~15ms)
```

---

## B3. 논문 실험 (Experiments)

### 3.1 데이터셋 준비
- [ ] Self-collected 5개 시퀀스 (S1-S5)
  - S1: Normal indoor (~20m)
  - S2: Long corridor (~30m)
  - S3: Fast rotation (>2 rad/s, ~15m)
  - S4: Glass / reflective (~15m)
  - S5: Bright outdoor (~20m)
- [ ] 로깅: raw camera (30Hz) + dToF depth+confidence (30Hz) + IMU (100Hz)
  - ARKit pose도 로깅 → baseline 비교 + ablation용
- [ ] 각 trajectory는 closed loop (start = end)
- [ ] ARKitScenes high-resolution subset (FARO S70 GT)
- [ ] Livox Mid-70 dense reference map (S1, S2, S4)

### 3.2 dToF Noise Characterization
- [ ] 5개 거리 (0.5, 1, 2, 3, 4m) × 100 static frames
- [ ] Confidence 별 plane fitting residual → σ(d, c) 측정
- [ ] Proposition 2의 η ≈ 1.54 실측 검증

### 3.3 Baseline 비교
- [ ] ARKit (platform VIO) — pose + reconstruction
- [ ] RTAB-Map (camera + dToF) — open-source multi-sensor
- [ ] Naive ICP (uniform weight, no CGD) — confidence 효과 격리
- [ ] **DV-SLAM full (camera + depth + IMU)** ← 논문 주인공
- [ ] FAST-LIO2 on dToF (ROS bridge) — dense LIO가 sparse에서 실패

### 3.4 Ablation Study (S1 기준)
- [ ] Full DV-SLAM (camera + depth + IMU, confidence + CGD + TLS)
- [ ] − Camera (depth + IMU only) — visual feature 기여 분리
- [ ] − Confidence (w_i = 1.0, uniform weight)
- [ ] − CGD → uniform voxel downsampling
- [ ] − TLS (robust kernel 제거)
- [ ] ~100 pts (stride 12) — sparse limit test
- [ ] + ARKit pose prior — 블랙박스 의존 vs 자체 pose 비교

### 3.5 Point-Count Sweep
- [ ] Stride {1, 4, 8, 12, 16} → ~{3000, 500, 200, 100, 50} points
- [ ] 각각 confidence ON/OFF
- [ ] Collapse boundary 식별
- [ ] Figure 생성

### 3.6 메트릭
- Trajectory: Loop closure drift (%), ATE RMSE (SE(3) aligned via evo)
- Map quality: Accuracy (cm), Completeness (cm), F-score (vs FARO/Livox GT)
- Runtime: 프레임당 처리시간 breakdown (IMU / feature tracking / ICP / total), peak memory

---

## B4. 논문 수정 필요 항목
- [ ] Abstract: multi-sensor tight coupling 서술, 블랙박스 의존 없음 명시
- [ ] Contributions 반영
- [ ] Section 3: System Overview — 3-sensor architecture (camera + dToF + IMU)
- [ ] Section 3.1: IMU propagation + static initialization (y-up)
- [ ] Section 3.x: Visual feature tracking (KLT + reprojection residual)
- [ ] Section 3.x: Tight coupling — depth ICP + visual reprojection in single ESKF update
- [ ] Section 3.5: Divergence guard → 공분산 기반
- [ ] Experiment: ablation에 −Camera 추가
- [ ] Discussion: ARKit 블랙박스 제거의 의의 (재현성, 이식성)

---

## B5. 논문 제출 체크리스트
- [ ] 모든 TODO 수치 채움
- [ ] Figure 1 (overview diagram) — 3-sensor pipeline
- [ ] Qualitative comparison figure
- [ ] Point-count sweep figure
- [ ] dToF noise characterization figure
- [ ] 코드에 ARKit VIO pose 의존 완전 제거 확인
- [ ] Visual feature tracking 구현 & 검증
- [ ] 코드-논문 일치 최종 검증
- [ ] 코드 공개 준비 (GitHub repo, README)
- [ ] RA-L 6+2 페이지 제한 확인
- [ ] IEEE format compliance
- [ ] References 완성

---

# 공통 사항

## 코드 WARNING 수정 (math.md Red-Team 보고서)

### Priority 3 (논문 리뷰어 지적 가능)
- [ ] [W-1] Iterated ESKF: P_pred 유지 방식으로 변경
- [ ] [W-14] min_observations 10 → 15-20 상향
- [ ] [W-13] TLS threshold를 ESKFOptions로 이동

### Priority 4 (방어적 코딩)
- [ ] [W-5] Large dt guard 0.5s → 0.05s 축소
- [ ] [W-9] distanceLimit <= 0 방어 추가
- [ ] [W-2] P_ eigenvalue clamping (floor 1e-12)
- [ ] [W-3] S condition number 체크

---

## 작업 순서 (권장)

```
═══ 병렬 진행 가능 ═══

Track A (프로덕트):                Track B (논문):
A1. 실기기 테스트                  B1. git checkout -b paper
A2. 고스트 수준 평가               B2. Step 1: ARKit VIO 제거 (A~F)
A3. IAP 등록                      B3. IMU+depth only 테스트 (중간 검증)
A4. App Store 제출                B4. Step 2: Visual feature 추가 (G~J)
                                  B5. Camera+depth+IMU 전체 테스트
                                  B6. 논문 수정
═══ Track B 계속 ═══              B7. 데이터 수집 & 실험
                                  B8. 논문 완성 & 제출
```

---

## 리스크 & 대비

1. **IMU-only (Step 1) 성능 부족**: 예상됨 → Step 2에서 camera 추가로 해결
2. **OpenCV 모바일 성능**: KLT는 경량, ~2ms/frame 예상. 무거우면 feature 수 줄이기
3. **Feature 초기화 depth 없음**: dToF가 sparse → 일부 feature에 depth 없을 수 있음
   - 대비: depth 있는 feature만 사용, 또는 multi-frame triangulation
4. **ARKit보다 성능 낮을 수 있음**: 정직하게 보고
   - "블랙박스 없이, 재현 가능한 방식으로 ARKit의 X% 달성" = contribution
5. **RA-L 8페이지에 다 담을 수 있나**: 3-sensor tight coupling + confidence + ablation
   - 대비: visual tracking은 간략히 (standard KLT), confidence 분석에 집중

---

# 논의 기록 (Key Decisions Log)

> 아래는 논문 방향에 대해 논의하며 내린 결정들. 나중에 논문 작성 시 참조.

## D1. "Camera-free" 프레이밍 — 기각
- 초기: camera-free LIO를 주장하려 했음
- 문제: 컬러 매핑에 이미 카메라 사용 → camera-free는 거짓
- 카메라 자체는 raw sensor data — 다른 SLAM 논문들도 다 카메라 씀 (R3LIVE, ORB-SLAM3 등)
- 카메라 사용이 novelty를 약화시키지 않음
- **결론: camera-free 버리고 multi-sensor tight coupling으로 전환**

## D2. ARKit VIO Pose = 블랙박스 — 제거 대상
- ARKit VIO pose는 Apple이 계산한 결과물 (raw data 아님)
- 논문에 서술 불가, 재현 불가 (Android에서 ARKit 없음)
- 리뷰어 지적: "성능이 ARKit 덕인지 당신 알고리즘 덕인지 분리 불가"
- **결론: ARKit에서 raw data만 사용 (camera, depth, confidence, IMU, intrinsics). VIO pose 제거.**

## D3. Pose Estimation 방식 — 분석적 ESKF+ICP+Visual
- pose estimation 자체는 검증된 표준 방법 (FAST-LIO2, DLIO 등 수천 인용)
- main contribution이 아님. contribution은 confidence-aware observation model
- **Learned pose (TLIO식) 기각 이유:**
  - ARKit을 GT로 학습 → teacher보다 나을 수 없음
  - 학습 데이터가 카메라 기반 → 독립성 주장 무력화
  - confidence vs network 기여 분리 불가
  - 분석적 방법이 해석 가능하고 재현 가능
- **결론: ESKF predict(IMU) + update(depth ICP + visual reprojection). 전부 분석적.**

## D4. Feature 3D 초기화의 장점
- dToF depth map이 있으므로 feature의 3D 위치를 바로 알 수 있음
- stereo matching이나 multi-frame triangulation 불필요
- depth + confidence 체크 후 바로 초기화 → 구현 매우 간단
- 이것도 dToF sensor의 장점으로 논문에 언급 가능

## D5. 좌표계 — y-up (ARKit convention)
- 현재 코드: g = (0, 0, -9.81) → z-up 가정 (잘못됨)
- ARKit은 y-up → 렌더링/SceneKit/PLY 내보내기 전부 y-up
- **결론: g = (0, -9.81, 0)으로 수정. y-up 유지. 변환 불필요.**
- 논문에 "y-up world convention (gravity = -y)" 한 줄 명시

## D6. 논문 핵심 Novelty — 센서 선택이 아니라 관측 모델
- novelty는 "어떤 센서를 쓰느냐"가 아님
- **"sparse dToF에서 confidence가 ICP를 살린다"가 핵심**
- 카메라 유무와 무관하게 depth 관측의 confidence weighting은 의미있음
- ablation: −Camera / −Confidence 둘 다 보여서 각 기여 분리

## D7. 프로덕트 vs 논문 분리
- 프로덕트: ARKit VIO pose 활용 (잘 작동하니까 건드리지 않음)
- 논문: 자체 pose estimation (블랙박스 제거, 재현 가능)
- 코드: main branch (프로덕트) / paper branch (논문)
- 논문 완성 후 자체 pose가 충분히 좋으면 프로덕트도 전환 검토

## D8. ARKit의 역할 정의
- ARKit = Hardware Abstraction Layer (센서 HAL)
- 제공하는 것: camera frame, depth map, confidence map, IMU data, camera intrinsics
- 제공하지만 안 쓰는 것: VIO pose (블랙박스)
- 다른 플랫폼 이식 시: 센서 인터페이스만 교체하면 됨 (알고리즘 변경 불필요)
- 논문 Discussion에서 이식성 언급

## D9. 성능 기대치
- ARKit VIO보다 정확할 가능성 낮음 (ARKit = 카메라 feature 수천개 + Apple 최적화)
- 하지만 이길 필요 없음: "블랙박스 없이 ARKit의 X% 달성" = 충분한 contribution
- 특정 시나리오(저조도, 텍스처 없는 벽)에서는 이길 수도 있음
- 정직하게 보고 — ablation에서 +ARKit prior도 보여줌

## D10. 시스템 명칭
- 엄밀히는 SLAM이 아닌 LIO (loop closure 없음)
- 하지만 DV-SLAM 이름 유지 (브랜딩)
- 논문에서는 "LiDAR-Inertial Odometry" 또는 "multi-sensor odometry"로 서술
