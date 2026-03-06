//
//  InterstitialAdManager.swift
//  DepthViz
//
//  보상형 전면 광고 관리 — 20회 이상 미구매 시 매 스캔 후 표시
//  끝까지 시청해야만 프리뷰 이동 가능
//

import UIKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

final class InterstitialAdManager: NSObject {

    static let shared = InterstitialAdManager()

    #if canImport(GoogleMobileAds)
    private var rewardedInterstitialAd: GADRewardedInterstitialAd?
    #endif
    private var isLoadingAd = false
    private var rewardEarned = false
    private var dismissCompletion: (() -> Void)?

    // MARK: - 광고 ID
    #if DEBUG
    private let adUnitID = "ca-app-pub-3940256099942544/6978759866" // Google 테스트 보상형 전면 ID
    #else
    private let adUnitID = "ca-app-pub-2516597008794244/5477223074" // 실제 보상형 전면 ID
    #endif

    private override init() {
        super.init()
    }

    /// 광고 미리 로드
    func loadAd() async {
        #if canImport(GoogleMobileAds)
        if isLoadingAd || rewardedInterstitialAd != nil { return }

        isLoadingAd = true

        do {
            rewardedInterstitialAd = try await GADRewardedInterstitialAd.load(
                withAdUnitID: adUnitID,
                request: GADRequest()
            )
            rewardedInterstitialAd?.fullScreenContentDelegate = self
            print("✅ RewardedInterstitialAd: 광고 로드 성공")
        } catch {
            print("❌ RewardedInterstitialAd: 광고 로드 실패 - \(error.localizedDescription)")
            rewardedInterstitialAd = nil
        }

        isLoadingAd = false
        #endif
    }

    /// 광고 표시 — 시청 완료(보상 획득) 후에만 completion 호출
    func showAd(from viewController: UIViewController, completion: @escaping () -> Void) {
        #if canImport(GoogleMobileAds)
        guard let ad = rewardedInterstitialAd else {
            print("⚠️ RewardedInterstitialAd: 준비 안 됨, 바로 진행")
            completion()
            Task { await loadAd() }
            return
        }

        rewardEarned = false
        dismissCompletion = completion

        ad.present(fromRootViewController: viewController) { [weak self] in
            // 보상 획득 — 끝까지 시청 완료
            print("🎁 RewardedInterstitialAd: 보상 획득 (시청 완료)")
            self?.rewardEarned = true
        }
        #else
        completion()
        #endif
    }

    var isAdAvailable: Bool {
        #if canImport(GoogleMobileAds)
        return rewardedInterstitialAd != nil
        #else
        return false
        #endif
    }
}

#if canImport(GoogleMobileAds)
// MARK: - GADFullScreenContentDelegate
extension InterstitialAdManager: GADFullScreenContentDelegate {

    func ad(_ ad: GADFullScreenPresentingAd, didFailToPresentFullScreenContentWithError error: Error) {
        print("❌ RewardedInterstitialAd: 표시 실패 - \(error.localizedDescription)")
        rewardedInterstitialAd = nil
        dismissCompletion?()
        dismissCompletion = nil
        Task { await loadAd() }
    }

    func adDidDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("✅ RewardedInterstitialAd: 광고 닫힘 (보상 획득: \(rewardEarned))")
        rewardedInterstitialAd = nil

        if rewardEarned {
            // 시청 완료 → 10분 쿨다운 시작 + 프리뷰 이동
            ScanCountManager.shared.markRewardedShown()
            dismissCompletion?()
        } else {
            // 시청 미완료 → 다시 광고 로드 후 재시도 (프리뷰 차단)
            print("⚠️ 시청 미완료 — 프리뷰 차단, 다음 광고 로드")
            Task { await loadAd() }
            // completion 호출 안 함 → 프리뷰 안 열림
        }
        dismissCompletion = nil
    }
}
#endif
