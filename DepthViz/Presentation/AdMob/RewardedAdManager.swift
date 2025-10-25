//
//  RewardedAdManager.swift
//  DepthViz
//
//  보상형 광고 관리 클래스
//

import UIKit
import GoogleMobileAds

/// 보상형 광고를 관리하는 매니저 클래스
final class RewardedAdManager: NSObject {
    
    // MARK: - Properties
    
    static let shared = RewardedAdManager()
    
    /// 보상형 광고 객체
    private var rewardedAd: GADRewardedAd?
    
    /// 광고 로딩 상태
    private var isLoadingAd = false
    
    /// 델리게이트
    weak var delegate: RewardedAdManagerDelegate?
    
    // MARK: - 광고 ID 설정
    #if DEBUG
    // 테스트용 광고 ID
    private let rewardedAdUnitID = "ca-app-pub-3940256099942544/1712485313" // Google 테스트 보상형 광고 ID
    #else
    // 실제 광고 ID (배포용)
    private let rewardedAdUnitID = "ca-app-pub-2516597008794244/2163101273"
    #endif
    
    private override init() {
        super.init()
    }
    
    // MARK: - Public Methods
    
    /// 광고 로드 (비동기)
    func loadAd() async {
        // 이미 광고가 있거나 로딩 중이면 중단
        if isLoadingAd || rewardedAd != nil {
            print("⏳ RewardedAd: 이미 광고가 있거나 로딩 중입니다.")
            return
        }
        
        isLoadingAd = true
        print("🔄 RewardedAd: 광고 로드 시작...")
        
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
    }
    
    /// 광고 표시 (가능한 경우)
    /// - Parameters:
    ///   - viewController: 광고를 표시할 View Controller
    ///   - completion: 보상 지급 완료 시 호출되는 클로저
    func showAdIfAvailable(from viewController: UIViewController, completion: @escaping (Bool, Int) -> Void) {
        guard let rewardedAd = rewardedAd else {
            print("⚠️ RewardedAd: 광고가 준비되지 않았습니다.")
            completion(false, 0)
            // 새 광고 로드
            Task {
                await loadAd()
            }
            return
        }
        
        print("📱 RewardedAd: 광고 표시 시작")
        
        rewardedAd.present(fromRootViewController: viewController) {
            let reward = rewardedAd.adReward
            print("🎁 RewardedAd: 보상 지급 - \(reward.amount) \(reward.type)")
            
            // 보상 지급
            completion(true, reward.amount.intValue)
        }
    }
    
    /// 광고 사용 가능 여부 확인
    func isAdAvailable() -> Bool {
        return rewardedAd != nil
    }
}

// MARK: - GADFullScreenContentDelegate

extension RewardedAdManager: GADFullScreenContentDelegate {
    
    func adDidRecordImpression(_ ad: GADFullScreenPresentingAd) {
        print("📊 RewardedAd: 광고 노출 기록됨")
    }
    
    func adDidRecordClick(_ ad: GADFullScreenPresentingAd) {
        print("👆 RewardedAd: 광고 클릭 기록됨")
    }
    
    func ad(_ ad: GADFullScreenPresentingAd, didFailToPresentFullScreenContentWithError error: Error) {
        print("❌ RewardedAd: 광고 표시 실패 - \(error.localizedDescription)")
        rewardedAd = nil
        delegate?.rewardedAdManagerAdDidComplete(self)
        
        // 새 광고 로드 시도
        Task {
            await loadAd()
        }
    }
    
    func adWillPresentFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("📱 RewardedAd: 광고 표시 시작")
    }
    
    func adWillDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("👋 RewardedAd: 광고 닫기 시작")
    }
    
    func adDidDismissFullScreenContent(_ ad: GADFullScreenPresentingAd) {
        print("✅ RewardedAd: 광고 닫힘 완료")
        rewardedAd = nil
        delegate?.rewardedAdManagerAdDidComplete(self)
        
        // 다음 광고 미리 로드
        Task {
            await loadAd()
        }
    }
}

// MARK: - Delegate Protocol

protocol RewardedAdManagerDelegate: AnyObject {
    /// 보상형 광고 생명주기가 완료되었을 때 호출
    func rewardedAdManagerAdDidComplete(_ rewardedAdManager: RewardedAdManager)
}

