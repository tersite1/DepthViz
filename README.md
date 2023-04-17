# PROJECT
iPhone LiDAR 센서를 통해 실내지도 구축을 위한 측정앱 및 뷰어 Tool
## 주요 기능
- LiDAR & GPS 측정 및 전송
- 2D 지도기반 LiDAR 데이터 3D 렌더링 및 표시
- LiDAR 데이터 가공 및 Annotation

[Wiki 참고](https://github.com/FreeDeveloper97/point-cloud-with-lidar/wiki)

---

# APPLICATION
## 개발 범위
- **LiDAR 측정**
- **GPS 측정**
- **GPS 기반 주소검색 API 활용 건물정보 조회**
- **LiDAR 및 데이터 전송**
- **측정된 LiDAR 데이터 리스트 수신**
- **LiDAR 데이터 상세내역 표시**
- **2D 지도 및 LiDAR 데이터 측정위치 표시**

## 개발 환경
- MacBook Pro(16형, 2021년 모델)
- MacOS Monterey 12.6.3
- iPhone 13 Pro (iOS 16.3 +)

## 사용 기술 (프레임워크)
**Xcode 14.2** 
- Swift Compiler: Swift 5
- Deployment: iOS 14.0 +
          

Library
- **Metal**
- **MetalKit**
- **ARKit**
- **CoreLocation**
- **MapKit**
- **Almofire**: 5.6.4

---

# INSTALL
1. Install Xcode
[developer.apple.com](https://developer.apple.com/download/all/?q=Xcode)

2. Clone
```bash
git clone https://github.com/FreeDeveloper97/point-cloud-with-lidar.git
```

3. Move to /SceneDepthPointCloud/Configuration
``` bash
cd point-cloud-with-lidar/SceneDepthPointCloud/Configuration
```

4. Add `env.xcconfig` file
Or, Create Configuration Settings File Save AS: `env.xcconfig`

```text
//
//  env.xcconfig
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

// Configuration settings file format documentation can be found at:
// https://help.apple.com/xcode/#/dev745c5c974

// 서버 URL
BASE_URL = http:/$()/121.172.188.145:3000

```

5. Connect Apple Device (LiDAR capable device, such as the iPad Pro devices and iPhone Pro devices)

6. Run Project
※ You can only run this Project in your connected Apple Device
※ You have to need Apple Developer Membership

---

# MAKERS
### 타이퍼스
인하대학교 컴퓨터공학과 컴퓨터공학 종합설계 [202301-CSE-001]
- 12171633 서형중(Web Front)
- 12171571 강민상(iOS App)
- 12181568 권현준(Backend)
