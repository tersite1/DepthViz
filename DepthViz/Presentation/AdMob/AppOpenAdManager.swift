//
//  AppOpenAdManager.swift
//  DepthViz
//
//  앱 오프닝 광고 관리 클래스
//

import UIKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

// MARK: - Delegate Protocol

protocol AppOpenAdManagerDelegate: AnyObject {
    func appOpenAdManagerAdDidComplete(_ appOpenAdManager: AppOpenAdManager)
}

/// 앱 오프닝 광고를 관리하는 매니저 클래스
final class AppOpenAdManager: NSObject {

    static let shared = AppOpenAdManager()

    #if canImport(GoogleMobileAds)
    private var appOpenAd: GADAppOpenAd?
    #endif
    private var isLoadingAd = false
    private var isShowingAd = false
    private var loadTime: Date?
    private let timeoutInterval: TimeInterval = 4 * 3600
    private let minimumLaunchCountForAd = 1

    weak var delegate: AppOpenAdManagerDelegate?

    #if DEBUG
    private let appOpenAdUnitID = "ca-app-pub-3940256099942544/5575463023"
    #else
    private let appOpenAdUnitID = "ca-app-pub-3940256099942544/5575463023" // 테스트 ID (교체 필요)
    #endif

    private override init() {
        super.init()
    }

    func loadAd() async {
        #if canImport(GoogleMobileAds)
        if isLoadingAd || isAdAvailable() { return }

        isLoadingAd = true

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
        #endif
    }

    func showAdIfAvailable() {
        #if canImport(GoogleMobileAds)
        if isShowingAd { return }

        if !isAdAvailable() {
            delegate?.appOpenAdManagerAdDidComplete(self)
            Task { await loadAd() }
            return
        }

        if !shouldShowAd() {
            delegate?.appOpenAdManagerAdDidComplete(self)
            return
        }

        if let appOpenAd = appOpenAd {
            guard let rootViewController = getRootViewController() else { return }
            isShowingAd = true
            appOpenAd.present(fromRootViewController: rootViewController)
        }
        #else
        delegate?.appOpenAdManagerAdDidComplete(self)
        #endif
    }

    private func isAdAvailable() -> Bool {
        #if canImport(GoogleMobileAds)
        return appOpenAd != nil && wasLoadTimeLessThanNHoursAgo(timeoutInterval: timeoutInterval)
        #else
        return false
        #endif
    }

    private func wasLoadTimeLessThanNHoursAgo(timeoutInterval: TimeInterval) -> Bool {
        guard let loadTime = loadTime else { return false }
        return Date().timeIntervalSince(loadTime) < timeoutInterval
    }

    private func shouldShowAd() -> Bool {
        return UserDefaults.standard.integer(forKey: "app_launch_count") >= minimumLaunchCountForAd
    }

    private func getRootViewController() -> UIViewController? {
        if let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
           let window = scene.windows.first(where: { $0.isKeyWindow }) {
            return window.rootViewController
        }
        return nil
    }
}

#if canImport(GoogleMobileAds)
extension AppOpenAdManager: GADFullScreenContentDelegate {

    func adDidRecordImpression(_ ad: GADFullScreenPresentingAd) {}
    func adDidRecordClick(_ ad: GADFullScreenPresentingAd) {}

    func ad(_ ad: GADFullScreenPresentingAd, didFailToPresentFullScreenContentWithError error: Error) {
        appOpenAd = nil
        isShowingAd = false
        delegate?.appOpenAdManagerAdDidComplete(self)
        Task { await loadAd() }
    }

    func adWillPresentFullScreenContent(_ ad: GADFullScreenPresentingAd) {}
    func adWillDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {}

    func adDidDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        appOpenAd = nil
        isShowingAd = false
        delegate?.appOpenAdManagerAdDidComplete(self)
        Task { await loadAd() }
    }
}
#endif

