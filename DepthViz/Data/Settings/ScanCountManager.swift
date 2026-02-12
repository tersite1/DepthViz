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
    private let premiumThreshold = 3   // 첫 팝업 기준 (3회)
    private let repeatInterval = 3     // 이후 3회마다 반복

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
        guard !PremiumManager.shared.isPremium else { return false }
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

    /// 스캔 횟수 리셋 (디버깅용)
    func reset() {
        UserDefaults.standard.removeObject(forKey: scanCountKey)
        UserDefaults.standard.removeObject(forKey: lastPromptCountKey)
        print("🔄 스캔 횟수 리셋")
    }
}
