# 🛠️ DepthViz 설정 및 빌드 가이드

## 📋 목차
1. [필수 요구사항](#필수-요구사항)
2. [프로젝트 설정](#프로젝트-설정)
3. [CocoaPods 설치](#cocoapods-설치)
4. [Xcode 설정](#xcode-설정)
5. [AdMob 설정](#admob-설정)
6. [빌드 및 실행](#빌드-및-실행)
7. [문제 해결](#문제-해결)

---

## 필수 요구사항

### 하드웨어
- ✅ LiDAR 센서가 있는 iPhone/iPad
  - iPhone 12 Pro 이상
  - iPhone 13 Pro 이상
  - iPhone 14 Pro 이상
  - iPhone 15 Pro 이상
  - iPad Pro (4세대) 이상

### 소프트웨어
- macOS Big Sur (11.0) 이상
- Xcode 13.0 이상
- iOS 14.0 이상 타겟
- CocoaPods 1.11.0 이상

---

## 프로젝트 설정

### 1단계: 프로젝트 다운로드/복사
현재 프로젝트 위치:
```
/Users/caz/Downloads/DepthViz_Final 3
```

### 2단계: Xcode에 새 파일 추가

Xcode에서 프로젝트를 열고 다음 파일들을 추가해야 합니다:

#### Domain 폴더
1. **PLYExporter.swift**
   - 경로: `DepthViz/Domain/PLYExporter.swift`
   - 타겟: DepthViz
   
2. **MetalTextureManager.swift**
   - 경로: `DepthViz/Domain/Metal/MetalTextureManager.swift`
   - 타겟: DepthViz

#### Presentation 폴더
3. **AdMobManager.swift**
   - 경로: `DepthViz/Presentation/AdMob/AdMobManager.swift`
   - 타겟: DepthViz

**파일 추가 방법:**
1. Xcode에서 프로젝트 네비게이터 열기
2. 해당 폴더에서 우클릭 → "Add Files to DepthViz..."
3. 파일 선택 후 "Copy items if needed" 체크
4. "Add" 클릭

---

## CocoaPods 설치

### 3단계: CocoaPods 설치 (처음 사용하는 경우)

```bash
# CocoaPods 설치 확인
pod --version

# 설치되지 않은 경우
sudo gem install cocoapods
```

### 4단계: 프로젝트 의존성 설치

```bash
# 프로젝트 디렉토리로 이동
cd "/Users/caz/Downloads/DepthViz_Final 3"

# CocoaPods 업데이트
pod repo update

# 의존성 설치
pod install
```

설치 완료 후 다음 메시지가 표시됩니다:
```
Pod installation complete! There is 1 dependency from the Podfile.
```

⚠️ **중요:** 설치 후 반드시 `.xcworkspace` 파일을 열어야 합니다!
```bash
open DepthViz.xcworkspace
```

---

## Xcode 설정

### 5단계: 빌드 설정 확인

1. Xcode에서 프로젝트 선택
2. TARGETS → DepthViz 선택
3. "Build Settings" 탭

확인할 설정:
- **iOS Deployment Target**: 14.0 이상
- **Swift Language Version**: Swift 5.0
- **Enable Bitcode**: No (AdMob 호환성)

### 6단계: Signing & Capabilities

1. "Signing & Capabilities" 탭
2. Team 선택 (Apple Developer 계정)
3. Bundle Identifier 설정 (예: com.yourcompany.depthviz)

필요한 Capabilities:
- ✅ Camera (자동 추가됨)
- ✅ Location (자동 추가됨)

### 7단계: Info.plist 설정

`Info.plist` 파일에 다음 항목 추가:

```xml
<!-- Camera 사용 권한 -->
<key>NSCameraUsageDescription</key>
<string>DepthViz uses the camera to capture 3D point cloud data using LiDAR.</string>

<!-- 위치 정보 사용 권한 -->
<key>NSLocationWhenInUseUsageDescription</key>
<string>DepthViz uses your location to tag scan data with GPS coordinates.</string>

<!-- ARKit 필요 -->
<key>UIRequiredDeviceCapabilities</key>
<array>
    <string>arkit</string>
</array>
```

AdMob 설정 후 추가할 항목 (6단계 참조):
```xml
<!-- AdMob App ID -->
<key>GADApplicationIdentifier</key>
<string>ca-app-pub-XXXXXXXX~YYYYYYYYYY</string>

<!-- SKAdNetwork IDs -->
<key>SKAdNetworkItems</key>
<array>
    <dict>
        <key>SKAdNetworkIdentifier</key>
        <string>cstr6suwn9.skadnetwork</string>
    </dict>
</array>
```

---

## 빌드 및 실행

### 10단계: 빌드

1. Xcode에서 타겟 디바이스 선택 (LiDAR 지원 기기)
2. Product → Build (⌘ + B)
3. 빌드 성공 확인

### 11단계: 실행

1. LiDAR 지원 iPhone/iPad를 Mac에 연결
2. 디바이스에서 개발자 모드 활성화
3. Product → Run (⌘ + R)

**첫 실행 시 권한 요청:**
- 카메라 접근 허용
- 위치 정보 접근 허용

---

## 문제 해결

### CocoaPods 관련

**문제:** `pod install` 실패
```bash
# 해결 방법 1: CocoaPods 캐시 삭제
pod cache clean --all
pod install

# 해결 방법 2: Repo 업데이트
pod repo update
pod install

# 해결 방법 3: Podfile.lock 삭제
rm Podfile.lock
pod install
```

### Xcode 빌드 오류

**문제:** "No such module GoogleMobileAds"
```
해결: 
1. Xcode 종료
2. pod install 재실행
3. .xcworkspace 파일 열기 (NOT .xcodeproj)
4. Clean Build Folder (⌘ + Shift + K)
5. 빌드 재시도
```

**문제:** "Undefined symbols for architecture arm64"
```
해결:
1. Build Settings → Enable Bitcode → No
2. Build Settings → Valid Architectures → arm64만 추가
3. Clean Build Folder
4. 빌드 재시도
```

### 런타임 오류

**문제:** "This app requires a LiDAR sensor"
```
해결:
- LiDAR 지원 기기에서만 실행 가능
- iPhone 12 Pro 이상 또는 iPad Pro (4세대) 이상 필요
```

**문제:** 광고가 표시되지 않음
```
해결:
1. 인터넷 연결 확인
2. AdMob 계정 활성화 확인
3. Info.plist에 GADApplicationIdentifier 추가 확인
4. 테스트 기기 등록 (AdMob 콘솔)
```

### 성능 문제

**문제:** 앱이 느리거나 메모리 부족
```
해결:
1. 포인트 수 줄이기 (maxPoints 조정)
2. 샘플링 포인트 줄이기 (numGridPoints 조정)
3. 백그라운드 앱 종료
4. 기기 재시작
```

---

## 체크리스트

개발 환경 설정:
- [ ] Xcode 설치 및 업데이트
- [ ] CocoaPods 설치
- [ ] LiDAR 지원 기기 준비
- [ ] Apple Developer 계정 설정

프로젝트 설정:
- [ ] pod install 실행
- [ ] .xcworkspace 열기
- [ ] 새 파일들 Xcode에 추가
- [ ] Bundle Identifier 설정
- [ ] Signing 설정

AdMob 설정:
- [ ] 앱 및 광고 단위 등록
- [ ] Info.plist 수정
- [ ] AdMobManager.swift 수정

테스트:
- [ ] 빌드 성공 확인
- [ ] 실제 기기에서 실행
- [ ] LiDAR 스캔 기능 테스트
- [ ] 파일 저장/불러오기 테스트
- [ ] 광고 표시 확인

---

## 다음 단계

✅ 설정 완료 후:
1. `README_IMPROVEMENTS.md` 읽기 - 모든 개선사항 확인
2. `ADMOB_SETUP_GUIDE.md` 읽기 - 상세 AdMob 가이드
3. 앱 테스트 및 피드백
4. 앱스토어 제출 준비

---

## 지원

문제가 계속되면:
- Apple Developer Forums
- Stack Overflow

**Happy Coding! 🚀**

