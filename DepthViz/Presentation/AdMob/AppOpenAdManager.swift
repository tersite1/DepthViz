//
//  AppOpenAdManager.swift
//  DepthViz
//
//  앱 오프닝 광고 관리 클래스
//

import UIKit
import GoogleMobileAds

/// 앱 오프닝 광고를 관리하는 매니저 클래스
final class AppOpenAdManager: NSObject {
    
    // MARK: - Properties
    
    static let shared = AppOpenAdManager()
    
    /// 앱 오프닝 광고 객체
    private var appOpenAd: GADAppOpenAd?
    
    /// 광고 로딩 상태
    private var isLoadingAd = false
    
    /// 광고 표시 상태
    private var isShowingAd = false
    
    /// 광고 로드 시간 (만료 체크용)
    private var loadTime: Date?
    
    /// 광고 타임아웃 시간 (4시간)
    private let timeoutInterval: TimeInterval = 4 * 3600
    
    /// 앱 실행 횟수 (처음 몇 번은 광고 표시 안함)
    private let minimumLaunchCountForAd = 1 // 첫 실행부터 광고 표시
    
    /// 델리게이트
    weak var delegate: AppOpenAdManagerDelegate?
    
    // MARK: - 광고 ID 설정
    #if DEBUG
    // 테스트용 광고 ID
    private let appOpenAdUnitID = "ca-app-pub-3940256099942544/5575463023" // Google 테스트 앱 오프닝 광고 ID
    #else
    // 실제 광고 ID (배포용)
    // TODO: 앱스토어 출시 전에 본인의 AdMob 계정에서 발급받은 광고 ID로 교체하세요
    private let appOpenAdUnitID = "YOUR_ACTUAL_APP_OPEN_AD_UNIT_ID"
    #endif
    
    private override init() {
        super.init()
    }
    
    // MARK: - Public Methods
    
    /// 광고 로드 (비동기)
    func loadAd() async {
        // 이미 광고가 있거나 로딩 중이면 중단
        if isLoadingAd || isAdAvailable() {
            print("⏳ AppOpenAd: 이미 광고가 있거나 로딩 중입니다.")
            return
        }
        
        isLoadingAd = true
        print("🔄 AppOpenAd: 광고 로드 시작...")
        
        do {
            appOpenAd = try await GADAppOpenAd.load(
                withAdUnitID: appOpenAdUnitID,
                request: GADRequest()
            )
            appOpenAd?.fullScreenContentDelegate = self
            loadTime = Date()
            print("✅ AppOpenAd: 광고 로드 성공")
        } catch {
            print("❌ AppOpenAd: 광고 로드 실패 - \(error.localizedDescription)")
            appOpenAd = nil
            loadTime = nil
        }
        
        isLoadingAd = false
    }
    
    /// 광고 표시 (가능한 경우)
    func showAdIfAvailable() {
        // 이미 광고를 표시 중이면 중단
        if isShowingAd {
            print("⚠️ AppOpenAd: 이미 광고를 표시 중입니다.")
            return
        }
        
        // 광고가 준비되지 않은 경우
        if !isAdAvailable() {
            print("⚠️ AppOpenAd: 광고가 준비되지 않았습니다.")
            delegate?.appOpenAdManagerAdDidComplete(self)
            // 새 광고 로드
            Task {
                await loadAd()
            }
            return
        }
        
        // 최소 실행 횟수 체크
        if !shouldShowAd() {
            print("⏭️ AppOpenAd: 아직 광고 표시 조건을 만족하지 않습니다.")
            delegate?.appOpenAdManagerAdDidComplete(self)
            return
        }
        
        // 광고 표시
        if let appOpenAd = appOpenAd {
            print("📱 AppOpenAd: 광고 표시 시작")
            
            // 루트 뷰 컨트롤러 가져오기
            guard let rootViewController = getRootViewController() else {
                print("❌ AppOpenAd: 루트 뷰 컨트롤러를 찾을 수 없습니다.")
                return
            }
            
            isShowingAd = true
            appOpenAd.present(fromRootViewController: rootViewController)
        }
    }
    
    // MARK: - Private Methods
    
    /// 광고 사용 가능 여부 확인
    private func isAdAvailable() -> Bool {
        return appOpenAd != nil && wasLoadTimeLessThanNHoursAgo(timeoutInterval: timeoutInterval)
    }
    
    /// 광고 로드 후 일정 시간이 지났는지 확인 (만료 체크)
    private func wasLoadTimeLessThanNHoursAgo(timeoutInterval: TimeInterval) -> Bool {
        guard let loadTime = loadTime else {
            return false
        }
        let timeElapsed = Date().timeIntervalSince(loadTime)
        return timeElapsed < timeoutInterval
    }
    
    /// 광고를 표시할지 여부 결정
    private func shouldShowAd() -> Bool {
        let launchCount = UserDefaults.standard.integer(forKey: "app_launch_count")
        return launchCount >= minimumLaunchCountForAd
    }
    
    /// 루트 뷰 컨트롤러 가져오기
    private func getRootViewController() -> UIViewController? {
        // Scene 기반
        if let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
           let window = scene.windows.first(where: { $0.isKeyWindow }) {
            return window.rootViewController
        }
        
        // Legacy
        if let window = UIApplication.shared.windows.first(where: { $0.isKeyWindow }) {
            return window.rootViewController
        }
        
        return nil
    }
}

// MARK: - GADFullScreenContentDelegate

extension AppOpenAdManager: GADFullScreenContentDelegate {
    
    func adDidRecordImpression(_ ad: GADFullScreenPresentingAd) {
        print("📊 AppOpenAd: 광고 노출 기록됨")
    }
    
    func adDidRecordClick(_ ad: GADFullScreenPresentingAd) {
        print("👆 AppOpenAd: 광고 클릭 기록됨")
    }
    
    func ad(_ ad: GADFullScreenPresentingAd, didFailToPresentFullScreenContentWithError error: Error) {
        print("❌ AppOpenAd: 광고 표시 실패 - \(error.localizedDescription)")
        appOpenAd = nil
        isShowingAd = false
        delegate?.appOpenAdManagerAdDidComplete(self)
        
        // 새 광고 로드 시도
        Task {
            await loadAd()
        }
    }
    
    func adWillPresentFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("📱 AppOpenAd: 광고 표시 시작")
    }
    
    func adWillDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("👋 AppOpenAd: 광고 닫기 시작")
    }
    
    func adDidDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("✅ AppOpenAd: 광고 닫힘 완료")
        appOpenAd = nil
        isShowingAd = false
        delegate?.appOpenAdManagerAdDidComplete(self)
        
        // 다음 광고 미리 로드
        Task {
            await loadAd()
        }
    }
}

// MARK: - Delegate Protocol

protocol AppOpenAdManagerDelegate: AnyObject {
    /// 앱 오프닝 광고 생명주기가 완료되었을 때 호출 (닫힘 또는 표시 실패)
    func appOpenAdManagerAdDidComplete(_ appOpenAdManager: AppOpenAdManager)
}

