# 🔑 AdMob API Key 등록 완벽 가이드

## 📌 중요: AdMob에서 수익을 얻으려면 반드시 자신의 API Key를 등록해야 합니다!

테스트 광고 ID로는 **실제 수익이 발생하지 않습니다**. 본인의 AdMob 계정에서 발급받은 광고 ID로 교체해야 합니다.

---

## 🎯 1단계: Google AdMob 계정 생성

### 1. AdMob 웹사이트 접속
```
https://admob.google.com
```

### 2. Google 계정으로 로그인
- 기존 Google 계정 사용 가능
- 또는 새 Google 계정 생성

### 3. AdMob 계정 생성
1. "시작하기" 버튼 클릭
2. 국가/지역 선택: **대한민국**
3. 이용약관 동의
4. 계정 생성 완료

---

## 📱 2단계: 앱 등록하기

### 1. 앱 추가
1. AdMob 콘솔 → 왼쪽 메뉴 → **"앱"** 클릭
2. **"앱 추가"** 버튼 클릭

### 2. 앱 정보 입력
- **앱이 이미 게시되어 있나요?**: **"아니요"** 선택
- **플랫폼**: **iOS** 선택
- **앱 이름**: **DepthViz** 입력
- **앱 추가** 버튼 클릭

### 3. App ID 복사
앱 등록 후 다음과 같은 형식의 **App ID**가 발급됩니다:
```
ca-app-pub-1234567890123456~0987654321
```

⚠️ **이 App ID를 복사해두세요!**

---

## 🎬 3단계: 앱 오프닝 광고 단위 생성

### 1. 광고 단위 추가
1. 앱 대시보드에서 **"광고 단위"** 탭 클릭
2. **"광고 단위 추가"** 버튼 클릭

### 2. 광고 형식 선택
- **"앱 오프닝 광고"** 선택

### 3. 광고 단위 설정
- **광고 단위 이름**: `DepthViz_AppOpen` 입력
- **고급 설정**: 기본값 사용
- **"광고 단위 만들기"** 버튼 클릭

### 4. 광고 단위 ID 복사
다음과 같은 형식의 **광고 단위 ID**가 발급됩니다:
```
ca-app-pub-1234567890123456/1111111111
```

⚠️ **이 광고 단위 ID를 복사해두세요!**

---

## 🎯 4단계: 배너 광고 단위 생성

### 1. 광고 단위 추가
1. 같은 앱 대시보드에서 **"광고 단위 추가"** 다시 클릭

### 2. 광고 형식 선택
- **"배너"** 선택

### 3. 광고 단위 설정
- **광고 단위 이름**: `DepthViz_Banner` 입력
- **배너 크기**: **표준 배너 (320x50)** 선택
- **"광고 단위 만들기"** 버튼 클릭

### 4. 광고 단위 ID 복사
다음과 같은 형식의 **광고 단위 ID**가 발급됩니다:
```
ca-app-pub-1234567890123456/2222222222
```

⚠️ **이 광고 단위 ID를 복사해두세요!**

---

## 💻 5단계: Xcode 프로젝트에 ID 등록

이제 복사한 ID들을 코드에 입력합니다.

### 1. Info.plist 수정

`DepthViz/Info.plist` 파일을 열고 다음을 추가:

```xml
<!-- AdMob App ID (2단계에서 복사한 App ID) -->
<key>GADApplicationIdentifier</key>
<string>ca-app-pub-1234567890123456~0987654321</string>
```

⚠️ **YOUR_APP_ID를 실제 App ID로 교체하세요!**

**예시:**
```xml
<key>GADApplicationIdentifier</key>
<string>ca-app-pub-3123456789012345~1234567890</string>
```

### 2. AppOpenAdManager.swift 수정

파일 경로: `DepthViz/Presentation/AdMob/AppOpenAdManager.swift`

다음 부분을 찾아서 수정:

```swift
#else
// 실제 광고 ID (배포용)
private let appOpenAdUnitID = "YOUR_ACTUAL_APP_OPEN_AD_UNIT_ID"
#endif
```

**수정 후:**
```swift
#else
// 실제 광고 ID (배포용)
private let appOpenAdUnitID = "ca-app-pub-1234567890123456/1111111111"
#endif
```

### 3. AdMobManager.swift 수정

파일 경로: `DepthViz/Presentation/AdMob/AdMobManager.swift`

다음 부분을 찾아서 수정:

```swift
#else
// 실제 광고 ID (배포용)
private let bannerAdUnitID = "YOUR_ACTUAL_BANNER_AD_UNIT_ID"
#endif
```

**수정 후:**
```swift
#else
// 실제 광고 ID (배포용)
private let bannerAdUnitID = "ca-app-pub-1234567890123456/2222222222"
#endif
```

---

## 📝 ID 정리 요약

| 항목 | 형식 | 사용 위치 |
|------|------|----------|
| **App ID** | `ca-app-pub-XXXXXXXX~YYYYYYYYYY` | Info.plist |
| **앱 오프닝 광고 단위 ID** | `ca-app-pub-XXXXXXXX/1111111111` | AppOpenAdManager.swift |
| **배너 광고 단위 ID** | `ca-app-pub-XXXXXXXX/2222222222` | AdMobManager.swift |

---

## ✅ 6단계: 테스트 및 확인

### 개발 중 테스트

1. Xcode에서 Build Configuration을 **Debug**로 설정
2. 앱 실행
3. **테스트 광고**가 표시됨 (실제 ID가 아닌 테스트 ID 사용)

### 실제 광고 테스트 (배포 전)

1. Xcode에서 Build Configuration을 **Release**로 변경:
   - Product → Scheme → Edit Scheme...
   - Run → Build Configuration → **Release** 선택

2. 앱을 실제 기기에서 실행

3. **실제 광고**가 표시되는지 확인

⚠️ **주의사항:**
- 자신의 광고를 **절대 클릭하지 마세요!** (계정 정지 위험)
- 테스트 기기를 AdMob에 등록하는 것을 권장합니다

---

## 🔒 7단계: 테스트 기기 등록 (권장)

자신의 광고를 안전하게 테스트하려면:

### 1. 기기 ID 가져오기

앱을 실행하면 Xcode 콘솔에 다음과 같은 로그가 표시됩니다:
```
<Google> To get test ads on this device, set:
GADMobileAds.sharedInstance.requestConfiguration.testDeviceIdentifiers = @[ @"DEVICE_ID_HERE" ];
```

### 2. AdMob 콘솔에서 테스트 기기 추가

1. AdMob 콘솔 → **설정** → **테스트 기기**
2. **테스트 기기 추가** 클릭
3. 기기 ID 입력
4. 저장

이제 이 기기에서는 실제 광고 ID를 사용해도 테스트 광고가 표시됩니다.

---

## 💰 8단계: 결제 정보 입력 (수익 받기)

실제 수익을 받으려면 결제 정보를 입력해야 합니다.

### 1. 결제 설정
1. AdMob 콘솔 → **결제** 탭
2. **결제 정보 추가** 클릭

### 2. 필수 정보 입력
- **은행 계좌 정보** (수익 입금용)
- **세금 정보** (한국: 사업자등록번호 또는 주민등록번호)
- **주소 정보**

### 3. 결제 임계값
- 기본 임계값: **$100** (약 13만원)
- 수익이 임계값에 도달하면 자동으로 은행 계좌로 입금됩니다

### 4. 결제 주기
- **매월 21일** 기준으로 전월 수익 정산
- 예: 1월 1일~31일 수익 → 2월 21일 입금

---

## 🎯 9단계: 앱스토어 제출 전 최종 체크리스트

배포 전 반드시 확인하세요:

### 필수 확인 사항

- [ ] **Info.plist에 실제 App ID 입력**
  ```xml
  <key>GADApplicationIdentifier</key>
  <string>ca-app-pub-XXXXXXXX~YYYYYYYYYY</string>
  ```

- [ ] **AppOpenAdManager.swift에 실제 광고 단위 ID 입력**
  ```swift
  private let appOpenAdUnitID = "ca-app-pub-XXXXXXXX/1111111111"
  ```

- [ ] **AdMobManager.swift에 실제 광고 단위 ID 입력**
  ```swift
  private let bannerAdUnitID = "ca-app-pub-XXXXXXXX/2222222222"
  ```

- [ ] **Build Configuration을 Release로 설정**

- [ ] **실제 기기에서 광고 로드 테스트**

- [ ] **AdMob 콘솔에서 결제 정보 입력 완료**

### App Store Connect 설정

- [ ] **앱 정보 → 광고 식별자(IDFA) 사용**: **예** 선택
- [ ] **개인정보 보호 정책 URL 입력** (AdMob 사용 명시)

---

## 📊 10단계: 수익 확인 및 모니터링

### AdMob 콘솔에서 수익 확인

1. AdMob 콘솔 → **홈** 또는 **보고서**
2. 다음 지표 확인:
   - **예상 수익**: 실시간 수익
   - **노출수**: 광고가 표시된 횟수
   - **클릭수**: 광고가 클릭된 횟수
   - **CTR (클릭률)**: 클릭수 / 노출수
   - **eCPM**: 1,000회 노출당 수익

### 예상 수익 계산

**배너 광고:**
- 평균 eCPM: $0.5 ~ $2 (국가/카테고리별 차이)
- DAU 1,000명 × 세션당 3회 노출 = 3,000 노출/일
- 수익 = (3,000 / 1,000) × $1 = $3/일 = $90/월

**앱 오프닝 광고:**
- 평균 eCPM: $3 ~ $10 (배너보다 높음)
- DAU 1,000명 × 세션당 1회 = 1,000 노출/일
- 수익 = (1,000 / 1,000) × $5 = $5/일 = $150/월

**총 예상 수익:** $240/월 (DAU 1,000명 기준)

---

## ⚠️ 중요 공지 및 주의사항

### 절대 하지 말아야 할 것

❌ **자신의 광고를 클릭하지 마세요**
- 계정 정지 위험
- 테스트 기기를 등록하거나 테스트 광고 ID 사용

❌ **광고 클릭을 유도하지 마세요**
- "광고를 클릭해주세요" 등의 문구 금지
- Google 정책 위반으로 계정 정지

❌ **광고를 너무 많이 표시하지 마세요**
- 사용자 경험을 해치면 안 됩니다
- Google에서 권장하는 빈도 준수

### 권장사항

✅ **사용자 경험 우선**
- 광고가 앱 사용을 방해하지 않도록
- 적절한 위치와 빈도

✅ **정책 준수**
- [AdMob 프로그램 정책](https://support.google.com/admob/answer/6128543) 숙지
- 정기적으로 정책 업데이트 확인

✅ **품질 높은 앱 제작**
- 좋은 사용자 평가 → 더 많은 다운로드 → 더 많은 수익

---

## 🆘 문제 해결

### 광고가 표시되지 않음

**원인 1: App ID가 잘못됨**
```
해결: Info.plist의 GADApplicationIdentifier 확인
```

**원인 2: 광고 단위 ID가 잘못됨**
```
해결: AppOpenAdManager.swift, AdMobManager.swift의 ID 확인
```

**원인 3: 인터넷 연결 없음**
```
해결: Wi-Fi 또는 셀룰러 데이터 연결 확인
```

**원인 4: AdMob 계정 승인 대기 중**
```
해결: 앱 등록 후 24~48시간 대기 (승인 필요)
```

### 테스트 광고는 나오는데 실제 광고가 안 나옴

**원인: Build Configuration이 Debug로 설정됨**
```
해결:
1. Product → Scheme → Edit Scheme...
2. Run → Build Configuration → Release 선택
3. 앱 재빌드 및 실행
```

### 수익이 발생하지 않음

**원인 1: 테스트 기기에서 테스트 중**
```
해결: 다른 사용자의 실제 기기에서 테스트 필요
```

**원인 2: 실제 광고 ID가 아닌 테스트 ID 사용 중**
```
해결: 코드의 광고 단위 ID를 실제 ID로 교체
```

---

## 📚 추가 리소스

- [AdMob 공식 문서](https://developers.google.com/admob/ios/quick-start)
- [AdMob 정책](https://support.google.com/admob/answer/6128543)
- [광고 최적화 가이드](https://support.google.com/admob/answer/6128877)
- [결제 FAQ](https://support.google.com/admob/answer/2709856)

---

## 🎉 완료!

이제 AdMob 설정이 완료되었습니다. 

**다음 단계:**
1. 앱을 App Store에 제출
2. 사용자 획득
3. 수익 모니터링
4. 지속적인 앱 개선

**Happy Monetizing! 💰**

