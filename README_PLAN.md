# 🎯 DepthViz iOS SLAM 완전 개선 플랜

## 📊 요약

**현재 상태**: 빌드는 되지만 **실사용 불가능** (3가지 치명적 결함)
**목표**: 완전히 작동하는 iOS SLAM 앱 (1.5일 소요)
**기본 알고리즘**: DV-SLAM (당신의 커스텀 알고리즘)
**추가 알고리즘**: FastLIO2, FastLIVO2, DLIO, SuperLIO (선택 가능)

---

## 🔴 현재 문제점

| # | 문제 | 심각도 | 원인 | 해결 시간 |
|---|------|--------|------|---------|
| 1 | 스캔 포인트가 실제 위치와 안 맞음 | 🔴 치명 | 좌표 변환 오류 + 신뢰도 필터 부재 | 1.5h |
| 2 | Preview 렌더링 안 됨 | 🔴 치명 | MTKView-Renderer 연결 부재 | 1.5h |
| 3 | Cancel/Back 버튼 미작동 | 🟠 높음 | 이벤트 핸들러 미구현 | 0.5h |
| 4 | 메인 화면 복귀 시 초기화 안 됨 | 🟠 높음 | viewWillAppear 미구현 | 0.5h |
| 5 | Settings 비활성 | 🟡 중간 | UI 미완성 | 1h |

---

## 🚀 4단계 해결 플랜

### **Phase 0: 긴급 안정화 (1시간) ⚡ 오늘 완료해야 함**

✅ **P0-1: Cancel/Back 네비게이션 수정** (15분)
- `ScanPreviewVC`의 백버튼에 이벤트 핸들러 추가
- SLAM 엔진 정지 → 렌더러 초기화 → 네비게이션 복귀

✅ **P0-2: 메인 화면 복귀 시 초기화** (30분)
- `MainVC.viewWillAppear()` 완전 리셋 로직
- SLAM 정지, 렌더러 클리어, 버퍼 해제

✅ **P0-3: Preview 렌더링 연결** (15분)
- `ScanPreviewVC`에 MTKView 생성
- MTKViewDelegate 구현

**결과**: 기본 플로우 작동 (스캔 시작 → 취소 → 메인으로)

---

### **Phase 1: DV-SLAM 알고리즘 개선 (3시간)**

✅ **P1-1: 좌표 정규화** (45분)
- ARKit 깊이 좌표 → 월드 좌표 변환
- 물리적으로 불가능한 거리 제거 (0.05m ~ 5m)

✅ **P1-2: 신뢰도 필터** (45분)
- 신뢰도 0.5 미만 포인트 제거
- 아래 공식으로 정규화:
  ```cpp
  float conf = (float)conf_byte / 255.0f;
  if (conf < 0.5f) continue;
  ```

✅ **P1-3: 노이즈 제거** (45분)
- 이상치 필터링
- Statistical outlier removal

✅ **P1-4: 시각적 검증** (30분)
- 스캔 결과 진단 리포트
- 포인트 통계 출력

**결과**: 실제 위치와 일치하는 포인트 클라우드

---

### **Phase 2: Settings UI 구현 (2시간)**

✅ **P2-1: AlgorithmSelectionView 생성** (45분)
- SwiftUI 기반 설정 화면
- 5가지 알고리즘 선택 가능
- DV-SLAM 기본값

✅ **P2-2: 버튼 이벤트 연결** (30분)
- `MainVC.settingsButton` 활성화
- UIHostingController로 SwiftUI 뷰 표시

✅ **P2-3: 알고리즘 전환** (30분)
- ScanSettings 모델 개선
- UserDefaults 저장
- SLAMService.reloadSettings() 호출

✅ **P2-4: 신뢰도 슬라이더** (15분)
- 사용자가 필터 강도 조절 가능

**결과**: 완전히 작동하는 Settings UI

---

### **Phase 3: 렌더링 완성 (4시간)**

✅ **P3-1: Renderer ↔ SLAM 동기화** (1시간)
- `updateFromSLAM()` 구현
- SLAM 포인트를 Renderer 버퍼에 주입

✅ **P3-2: 렌더링 루프** (1시간)
- `MTKViewDelegate` 구현
- `CADisplayLink` 60 FPS 보장
- 매 프레임마다 `renderer.draw()` 호출

✅ **P3-3: SLAMService 개선** (1시간)
- `getDisplayCloud()` 메서드 공개
- Renderer가 주기적으로 호출

✅ **P3-4: Metal 셰이더 최적화** (1시간)
- 기존 unprojection 셰이더 최적화
- 신뢰도 기반 필터링 추가

**결과**: Preview에서 포인트 클라우드 실시간 렌더링

---

### **Phase 4: 테스트 & 최적화 (2시간)**

✅ **테스트 1**: DV-SLAM 기본 선택 확인
✅ **테스트 2**: 알고리즘 변경 후 재스캔
✅ **테스트 3**: Cancel → 메인 화면 복귀
✅ **테스트 4**: Preview 포인트 정확성
✅ **테스트 5**: 메모리 누수 검증

**결과**: 완전히 작동하는 상용 앱

---

## 📁 수정할 파일 목록

### Swift Files
```
Presentation/Main/MainVC.swift
  ├─ viewWillAppear() → 초기화 로직 추가
  └─ tapSettingsButton() → 이벤트 핸들러 구현

Presentation/ScanViewer/ScanPreviewVC.swift
  ├─ viewDidLoad() → MTKView 생성
  ├─ MTKViewDelegate 구현
  └─ handleBackButtonTap() → 네비게이션

Presentation/Settings/AlgorithmSelectionView.swift (신규)
  └─ SwiftUI 기반 Settings 뷰

Domain/Renderer.swift
  ├─ updateFromSLAM() 추가
  └─ SLAM 데이터 수신 로직

Data/Settings/ScanSettings.swift
  └─ selectedAlgorithm 추가
```

### C++ Files
```
Domain/Algorithm/DepthViz/src/DepthVizEngine.cpp
  ├─ pushPointCloud() → 필터링 로직 개선
  └─ bundleAndDiscard() → 좌표 정규화
```

### Objective-C++ Files
```
Domain/Algorithm/Bridge/SLAMService.mm
  └─ getDisplayCloud() 메서드 공개
```

---

## ⏱️ 예상 소요 시간

| Phase | 작업 | 시간 | 우선순위 |
|-------|------|------|---------|
| 0 | Emergency Fix | 1h | 🔴 오늘 |
| 1 | DV-SLAM 개선 | 3h | 🔴 내일 |
| 2 | Settings UI | 2h | 🟠 내일 |
| 3 | 렌더링 | 4h | 🟠 내일/모레 |
| 4 | 테스트 | 2h | 🟡 모레 |
| **합계** | | **12h** | **1.5 days** |

---

## 📋 오늘(TODAY) 완료 목표

1. ✅ Cancel 버튼 작동 (ScanPreviewVC)
2. ✅ 메인 화면 초기화 (MainVC)
3. ✅ Preview 렌더링 시작 (MTKView)

이 3가지만 완료되면 **기본 플로우가 작동**합니다.

---

## 📚 문서 구조

```
📄 QUICK_START.txt (이 파일)
   └─ 빠른 개요

📄 IMPLEMENTATION_PLAN.md
   ├─ Phase별 상세 계획
   ├─ 각 함수의 전체 코드 예제
   ├─ 체크리스트
   └─ 성공 기준

📄 TECHNICAL_ANALYSIS.md
   ├─ 각 문제의 근본 원인
   ├─ 좌표 변환 설명
   ├─ 필터링 로직 상세 분석
   └─ 검증 방법
```

---

## ✅ 최종 성공 기준

1. **정확도**: 스캔 포인트 ±5cm 이내
2. **렌더링**: Preview에 포인트 표시됨
3. **네비게이션**: Cancel → 메인 + 초기화
4. **다중 지원**: 5가지 알고리즘 선택 가능 (DV-SLAM 기본)
5. **안정성**: 메모리 누수 없음, 반복 스캔 가능

---

## 🎯 다음 단계

1. `IMPLEMENTATION_PLAN.md` 열기
2. Phase 0부터 시작 (Cancel 버튼 수정)
3. 각 Phase마다 체크리스트 표시
4. Phase 4에서 5가지 테스트 실행

**Let's go! 🚀**

---
Created: 2026-01-20
Status: Ready to implement
