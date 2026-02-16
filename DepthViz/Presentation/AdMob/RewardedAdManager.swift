//
//  RewardedAdManager.swift
//  DepthViz
//
//  보상형 광고 관리 클래스
//

import UIKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

// MARK: - Delegate Protocol

protocol RewardedAdManagerDelegate: AnyObject {
    func rewardedAdManagerAdDidComplete(_ rewardedAdManager: RewardedAdManager)
}

/// 보상형 광고를 관리하는 매니저 클래스
final class RewardedAdManager: NSObject {

    static let shared = RewardedAdManager()

    #if canImport(GoogleMobileAds)
    private var rewardedAd: GADRewardedAd?
    #endif
    private var isLoadingAd = false

    weak var delegate: RewardedAdManagerDelegate?

    #if DEBUG
    private let rewardedAdUnitID = "ca-app-pub-3940256099942544/1712485313"
    #else
    private let rewardedAdUnitID = "ca-app-pub-2516597008794244/2163101273"
    #endif

    private override init() {
        super.init()
    }

    func loadAd() async {
        #if canImport(GoogleMobileAds)
        if isLoadingAd || rewardedAd != nil { return }

        isLoadingAd = true

        do {
            rewardedAd = try await GADRewardedAd.load(
                withAdUnitID: rewardedAdUnitID,
                request: GADRequest()
            )
            rewardedAd?.fullScreenContentDelegate = self
            print("✅ RewardedAd: 광고 로드 성공")
        } catch {
            print("❌ RewardedAd: 광고 로드 실패 - \(error.localizedDescription)")
            rewardedAd = nil
        }

        isLoadingAd = false
        #endif
    }

    func showAdIfAvailable(from viewController: UIViewController, completion: @escaping (Bool, Int) -> Void) {
        #if canImport(GoogleMobileAds)
        guard let rewardedAd = rewardedAd else {
            completion(false, 0)
            Task { await loadAd() }
            return
        }

        rewardedAd.present(fromRootViewController: viewController) {
            let reward = rewardedAd.adReward
            completion(true, reward.amount.intValue)
        }
        #else
        completion(false, 0)
        #endif
    }

    func isAdAvailable() -> Bool {
        #if canImport(GoogleMobileAds)
        return rewardedAd != nil
        #else
        return false
        #endif
    }
}

#if canImport(GoogleMobileAds)
extension RewardedAdManager: GADFullScreenContentDelegate {
    func adDidRecordImpression(_ ad: GADFullScreenPresentingAd) {}
    func adDidRecordClick(_ ad: GADFullScreenPresentingAd) {}

    func ad(_ ad: GADFullScreenPresentingAd, didFailToPresentFullScreenContentWithError error: Error) {
        rewardedAd = nil
        delegate?.rewardedAdManagerAdDidComplete(self)
        Task { await loadAd() }
    }

    func adWillPresentFullScreenContent(_ ad: GADFullScreenPresentingAd) {}
    func adWillDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {}

    func adDidDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        rewardedAd = nil
        delegate?.rewardedAdManagerAdDidComplete(self)
        Task { await loadAd() }
    }
}
#endif

