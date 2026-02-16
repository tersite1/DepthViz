//
//  ScanCountManager.swift
//  DepthViz
//
//  스캔 횟수 관리 클래스 — 3회 이상 시 프리미엄 팝업, 이후 3회마다 반복
//

import Foundation

/// 스캔 횟수를 관리하는 매니저 클래스
final class ScanCountManager {
    static let shared = ScanCountManager()

    private let scanCountKey = "scan_count_total"
    private let lastPromptCountKey = "last_premium_prompt_count"
    #if DEBUG
    private let premiumThreshold = 1   // 디버그: 1회부터 팝업
    private let repeatInterval = 1     // 디버그: 매회 반복
    private let adThreshold = 1        // 디버그: 1회부터 광고
    #else
    private let premiumThreshold = 3   // 첫 팝업 기준 (3회)
    private let repeatInterval = 3     // 이후 3회마다 반복
    private let adThreshold = 10       // 전면 광고 기준 (10회)
    #endif

    private init() {}

    /// 현재 스캔 횟수
    var currentCount: Int {
        return UserDefaults.standard.integer(forKey: scanCountKey)
    }

    /// 스캔 횟수 증가
    func increment() {
        let newCount = currentCount + 1
        UserDefaults.standard.set(newCount, forKey: scanCountKey)
        print("📊 스캔 횟수 증가: \(newCount)건")
    }

    /// 프리미엄 구매 제안 팝업을 표시해야 하는지 확인
    /// - 3회 이상 && 미구매 && (첫 번째 또는 마지막 팝업 이후 3회 이상)
    var shouldShowPremiumPrompt: Bool {
        #if !DEBUG
        guard !PremiumManager.shared.isPremium else { return false }
        #endif
        guard currentCount >= premiumThreshold else { return false }

        let lastPromptCount = UserDefaults.standard.integer(forKey: lastPromptCountKey)

        // 한 번도 팝업을 보여준 적 없으면 → 표시
        if lastPromptCount == 0 {
            return true
        }

        // 마지막 팝업 이후 3회 이상 스캔했으면 → 표시
        return (currentCount - lastPromptCount) >= repeatInterval
    }

    /// 팝업 표시 완료 시 호출 — 현재 스캔 횟수 기록
    func markPromptShown() {
        UserDefaults.standard.set(currentCount, forKey: lastPromptCountKey)
        print("📊 프리미엄 팝업 표시 완료 (count: \(currentCount))")
    }

    /// 10회 이상 미구매 → 매 스캔마다 전면 광고 표시
    var shouldShowInterstitialAd: Bool {
        #if DEBUG
        // 디버그: 프리미엄이어도 광고 표시 (테스트용)
        return currentCount >= adThreshold
        #else
        guard !PremiumManager.shared.isPremium else { return false }
        return currentCount >= adThreshold
        #endif
    }

    /// 스캔 횟수 리셋 (디버깅용)
    func reset() {
        UserDefaults.standard.removeObject(forKey: scanCountKey)
        UserDefaults.standard.removeObject(forKey: lastPromptCountKey)
        print("🔄 스캔 횟수 리셋")
    }
}
