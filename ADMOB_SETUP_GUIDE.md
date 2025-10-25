# 📱 AdMob 통합 가이드

## 🎯 앱스토어 출시를 위한 AdMob 설정 단계

### 1단계: Google AdMob 계정 생성

1. [Google AdMob](https://admob.google.com) 방문
2. Google 계정으로 로그인
3. "시작하기" 클릭하여 계정 생성

### 2단계: 앱 등록

1. AdMob 콘솔에서 "앱" → "앱 추가" 클릭
2. "앱이 이미 게시되어 있나요?" → "아니요" 선택
3. 플랫폼: iOS 선택
4. 앱 이름: **DepthViz** 입력
5. "앱 추가" 클릭

### 3단계: 광고 단위 생성

1. 앱 대시보드에서 "광고 단위" → "광고 단위 추가" 클릭
2. "배너" 선택
3. 광고 단위 이름: **DepthViz_Banner** 입력
4. "광고 단위 만들기" 클릭
5. **광고 단위 ID를 복사** (예: ca-app-pub-XXXXXXXX/YYYYYYYYYY)

### 4단계: CocoaPods를 통한 SDK 설치

#### Podfile 생성 또는 수정

터미널에서 프로젝트 디렉토리로 이동:

```bash
cd "/Users/caz/Downloads/DepthViz_Final 3"
```

Podfile 생성 (없는 경우):

```bash
pod init
```

Podfile을 다음과 같이 수정:

```ruby
platform :ios, '14.0'
use_frameworks!

target 'DepthViz' do
  # Google Mobile Ads SDK
  pod 'Google-Mobile-Ads-SDK', '~> 10.14.0'
end

post_install do |installer|
  installer.pods_project.targets.each do |target|
    target.build_configurations.each do |config|
      config.build_settings['IPHONEOS_DEPLOYMENT_TARGET'] = '14.0'
    end
  end
end
```

CocoaPods 설치:

```bash
pod install
```

⚠️ **중요:** 설치 후 `.xcodeproj` 대신 `.xcworkspace` 파일을 열어야 합니다!

### 5단계: Info.plist 설정

`Info.plist` 파일에 AdMob App ID 추가:

```xml
<key>GADApplicationIdentifier</key>
<string>ca-app-pub-XXXXXXXX~YYYYYYYYYY</string>

<key>SKAdNetworkItems</key>
<array>
  <dict>
    <key>SKAdNetworkIdentifier</key>
    <string>cstr6suwn9.skadnetwork</string>
  </dict>
  <!-- Google AdMob SKAdNetwork IDs -->
  <dict>
    <key>SKAdNetworkIdentifier</key>
    <string>4fzdc2evr5.skadnetwork</string>
  </dict>
</array>
```

### 6단계: 코드에 광고 ID 입력

`AdMobManager.swift` 파일의 다음 부분을 수정:

```swift
#else
// 실제 광고 ID (배포용)
private let bannerAdUnitID = "ca-app-pub-XXXXXXXX/YYYYYYYYYY" // 3단계에서 복사한 광고 단위 ID
#endif
```

### 7단계: AppDelegate에서 초기화

`AppDelegate.swift`의 `application(_:didFinishLaunchingWithOptions:)` 메서드에 추가:

```swift
import GoogleMobileAds

func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
    // AdMob 초기화
    AdMobManager.shared.initializeAdMob {
        print("AdMob initialized successfully")
    }
    
    return true
}
```

### 8단계: MainVC에 배너 광고 추가

`MainVC.swift`의 `viewDidLoad()` 메서드에 추가:

```swift
override func viewDidLoad() {
    super.viewDidLoad()
    
    // 기존 코드...
    self.configureUI()
    self.configureViewModel()
    
    // 배너 광고 추가
    AdMobManager.shared.addBannerToViewController(self, at: .bottom)
}
```

## 🧪 테스트 방법

### 개발 중 테스트

현재 코드는 DEBUG 모드에서 자동으로 Google의 테스트 광고 ID를 사용합니다:
- 시뮬레이터나 실제 기기에서 앱 실행
- 하단에 테스트 광고 배너가 표시되는지 확인
- 테스트 광고에는 "Test Ad" 표시가 있습니다

### 실제 광고 테스트 (출시 전)

1. Xcode에서 Build Configuration을 "Release"로 변경
2. AdMobManager.swift의 실제 광고 ID 입력 확인
3. 실제 기기에서 테스트 (시뮬레이터는 실제 광고 미지원)

⚠️ **주의사항:**
- 자신의 광고를 클릭하지 마세요 (계정 정지 위험)
- 테스트 시에는 테스트 기기를 등록하세요

## 💰 수익화 활성화

1. AdMob 콘솔 → "결제" 섹션
2. 은행 계좌 정보 입력
3. 세금 정보 제출
4. 결제 임계값 도달 시 자동 입금

## 📊 수익 확인

- AdMob 콘솔 → "홈" 또는 "보고서"에서 실시간 수익 확인
- 일일 수익, 노출수, 클릭수 등 상세 통계 제공

## 🚀 앱스토어 출시 체크리스트

- [ ] CocoaPods로 Google-Mobile-Ads-SDK 설치
- [ ] Info.plist에 GADApplicationIdentifier 추가
- [ ] Info.plist에 SKAdNetworkItems 추가
- [ ] AdMobManager에 실제 광고 단위 ID 입력
- [ ] AppDelegate에서 AdMob 초기화
- [ ] MainVC에 배너 광고 추가
- [ ] 실제 기기에서 광고 로드 테스트
- [ ] AdMob 계정에서 결제 정보 입력
- [ ] App Store Connect에서 광고 설정 활성화

## 📚 추가 리소스

- [AdMob 공식 문서](https://developers.google.com/admob/ios/quick-start)
- [AdMob 정책](https://support.google.com/admob/answer/6128543)
- [iOS SDK 통합 가이드](https://developers.google.com/admob/ios/banner)

## ⚠️ 중요 공지

**테스트 광고 ID는 개발 중에만 사용하고, 앱스토어 출시 전에 반드시 실제 광고 ID로 교체해야 합니다!**

