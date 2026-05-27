# DV-SLAM RA-L 실험 계획

## 실험 구조: 2 테이블 + 1 Figure

| 항목 | 주장 | 형태 |
|------|------|------|
| Table III | sparse dToF에서 LIO 작동 | 3행 비교 |
| Table IV | confidence weighting 필수 | 5행 ablation |
| Fig. 5 | 정성적 품질 차이 | 3열 이미지 |
| 본문 | runtime, DG, weight sensitivity | 수치 1-2줄씩 |

---

## Table III — Main Comparison (3행)

```
Method              | ATE [cm] | Acc [cm] | F-score
-------------------------------------------------
ARKit VIO           |    --    |   --     |   --
Naive ICP (w=1)     |    --    |   --     |   --
DV-SLAM (full)      |    --    |   --     |   --
```

| Config | Preset | 근거 |
|--------|--------|------|
| ARKit VIO | `arkitBaseline` | 모바일 유일 대안. 사용자 비교 대상 |
| Naive ICP | `noConfidence` | "confidence 없어도 되지 않나?" 반론 차단 |
| DV-SLAM | `full` | 전체 파이프라인 |

**FAST-LIO2/LIO-SAM 미포함 사유**: dToF 입력 불가 (>10K pts 가정). 넣으면 unfair comparison.

---

## Table IV — Ablation (5행)

```
Configuration               | Acc [cm] | F-score | ATE [cm]
------------------------------------------------------------
Full DV-SLAM                |    --    |   --    |   --
- Confidence (w=1)          |    --    |   --    |   --
- CGD → uniform voxel DS    |    --    |   --    |   --
~100 pts (stride 12)        |    --    |   --    |   --
~3000 pts (all, stride 1)   |    --    |   --    |   --
```

| 행 | Preset | 검증 대상 |
|----|--------|-----------|
| Full | `full` | 기준선 |
| -Confidence | `noConfidence` | C2 직접 검증 (Gauss-Markov) |
| -CGD | `uniformDS` | CGD vs 단순 다운샘플링 |
| ~100 pts | `sparse100` | collapse boundary — 핵심 실험 |
| ~3000 pts | `dense3000` | "CGD가 정보 버리는 거 아니냐?" 반론 차단 |

---

## 본문 서술 (테이블 없이)

- **Runtime**: "ESKF predict 2ms, CGD 3ms, ICP 8ms, total <15ms at 10Hz on A16"
- **Divergence Guard**: "Without DG, N/M challenging sequences diverged within 30s"
- **Weight sensitivity**: "Performance stable across w_M ∈ [0.25, 0.75]"

---

## Fig. 5 — Qualitative

같은 방 3열 비교:
```
[ARKit recon] | [DV-SLAM no conf] | [DV-SLAM full]
```

---

## 데이터셋

### Option A: ARKitScenes (추천)
- https://github.com/apple/ARKitScenes
- iPad Pro dToF 촬영, 5047 captures
- **GT**: 정밀 레이저 스캐너 point cloud + mesh
- **confidence 맵 제공** (raw 데이터셋에서)
- 필요 다운로드: `confidence`, `lowres_depth`, `mesh`, `lowres_wide.traj`
- **주의**: 오프라인 재생 모드 필요 (현재 앱은 실시간 전용)

```bash
python3 download_data.py raw --split Training --video_id 47333462 \
  --download_dir /tmp/ARKitScenes/ \
  --raw_dataset_assets mesh confidence lowres_depth lowres_wide.traj \
  lowres_wide lowres_wide_intrinsics
```

### Option B: Self-collected (보조)
| ID | 환경 | 난이도 | 특징 |
|----|------|--------|------|
| S1 | 사무실 | Normal | 텍스처 풍부 |
| S2 | 복도 | Normal | 긴 직선 |
| S3 | 계단 | Normal | 높이 변화 |
| S4 | 어두운 방 | Challenging | 저조도 |
| S5 | 유리/반사 | Challenging | 멀티패스 |
| S6 | 빠른 이동 | Challenging | 모션 블러 |

GT: ARKit mesh를 proxy로 (한계 논문에 명시)

---

## 코드 연동

### ExperimentConfig.swift (생성 완료)
`DepthViz/Data/Settings/ExperimentConfig.swift`

6개 프리셋:
- `ExperimentConfig.full` — 전체 파이프라인
- `ExperimentConfig.noConfidence` — w=1 uniform
- `ExperimentConfig.uniformDS` — CGD 비활성화
- `ExperimentConfig.sparse100` — ~100 pts/frame
- `ExperimentConfig.dense3000` — ~3000 pts/frame
- `ExperimentConfig.arkitBaseline` — ARKit only

### Renderer.swift 연동 필요사항

1. `numGridPoints` → `ExperimentManager.shared.overrideGridPoints ?? (기존 로직)`
2. `cameraRotationThreshold` → override 적용
3. `cameraTranslationThreshold` → override 적용
4. `optimizeAndExport()` → `enableSurfaceThinning`, `enableOutlierRemoval` 분기
5. 내보내기 시 `ExperimentManager.shared.saveMetadata()` 호출

### Metric 계산 (Python)
- Accuracy: GT mesh 대비 exported PLY의 chamfer distance
- Completeness: GT mesh 포인트 중 매칭된 비율
- F-score: precision + recall @ threshold
- ATE: trajectory vs GT trajectory RMSE

---

## 실행 순서

1. Renderer.swift에 ExperimentManager 연동
2. ARKitScenes 다운로드 (5-10개 시퀀스)
3. 오프라인 재생 모드 구현 (또는 self-collected)
4. 6 preset × 6 sequence = 36 runs
5. Python metric 스크립트로 테이블 채우기
