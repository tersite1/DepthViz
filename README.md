# DepthViz

iOS LiDAR 포인트클라우드 스캐닝 앱 (ARKit + Metal)

## Requirements

- Xcode 15.0+
- iOS 14.0+
- LiDAR 탑재 iPhone/iPad (iPhone 12 Pro 이상)

## Setup

### 1. OpenCV Framework (필수)

DV-SLAM 알고리즘에 OpenCV가 필요합니다.

```bash
# opencv2.framework 다운로드 (iOS용, ~568MB)
# https://github.com/opencv/opencv/releases 에서 iOS pack 다운로드
# 또는 아래 직접 빌드:
# https://docs.opencv.org/4.x/d5/da3/tutorial_ios_install.html
```

다운로드 후 아래 두 경로에 배치:
```
DepthViz/Frameworks/opencv.framework
```
프로젝트 루트에도:
```
opencv.framework
```

### 2. CocoaPods (필수)

AdMob SDK 설치:
```bash
pod install
```
설치 후 `DepthViz.xcworkspace`로 프로젝트를 열어야 합니다 (`.xcodeproj` 아님).

### 3. 외부 참고 레포 (선택)

SLAM 알고리즘 참고용 (빌드에 불필요, 이미 소스 포함됨):
- [Super-LIO](https://github.com/xuankuzcr/Super-LIO-SAM) — LiDAR-Inertial Odometry
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) — Fast LiDAR-Visual-Inertial Odometry

## Build

1. `pod install`
2. `DepthViz.xcworkspace` 열기
3. Signing & Capabilities에서 Team 설정
4. Bundle ID: `DepthViz-Kaist`
5. LiDAR 기기에서 빌드 & 실행

## Project Structure

```
DepthViz/
├── Domain/
│   ├── Renderer.swift          # Metal 렌더링 (포인트클라우드)
│   ├── PointCloudExporter.swift # PLY 내보내기
│   ├── Algorithm/              # DV-SLAM (LIO 기반)
│   │   ├── Bridge/             # Swift ↔ C++ 브릿지
│   │   ├── ThirdParty/Eigen/   # 선형대수 라이브러리
│   │   └── ...
│   ├── Metal/                  # 셰이더, 버퍼
│   └── Premium/                # 인앱결제, 프리미엄 기능
├── Presentation/
│   ├── Main/                   # 메인 스캔 화면
│   ├── ScanViewer/             # 3D 프리뷰 (SceneKit)
│   ├── ScanStorageVC/          # 프로젝트/스캔 목록
│   └── Settings/               # 설정 (알고리즘, 프리미엄)
├── Data/                       # 저장소, 설정
├── Resources/                  # 데모 영상
└── Assets.xcassets/            # 이미지, 아이콘
```

## Features

- LiDAR 실시간 포인트클라우드 시각화 (Metal)
- DV-SLAM / ARKit 알고리즘 선택
- PLY (ASCII/Binary) 내보내기
- 3D SceneKit 프리뷰 + 터치 회전
- 프리미엄: IMU 데이터 오버레이, Odometry 트라젝토리, 앱 아이콘
- 다국어 지원 (한국어/영어)
