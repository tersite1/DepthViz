//
//  ScanCountManager.swift
//  DepthViz
//
//  스캔 횟수 + 저장 횟수 관리 — 광고 표시 조건 판단
//

import Foundation

final class ScanCountManager {
    static let shared = ScanCountManager()

    private let scanCountKey = "scan_count_total"
    private let saveCountKey = "save_count_total"
    private let adThreshold = 10       // 배너 광고: 스캔 10회 이상
    private let rewardedSaveThreshold = 5  // 보상형 광고: 저장 5회 이후
    private let rewardedCooldown: TimeInterval = 600  // 보상형 쿨다운: 10분

    private var lastRewardedTime: Date?

    private init() {}

    // MARK: - Scan Count

    var currentCount: Int {
        return UserDefaults.standard.integer(forKey: scanCountKey)
    }

    func increment() {
        let newCount = currentCount + 1
        UserDefaults.standard.set(newCount, forKey: scanCountKey)
        print("📊 스캔 횟수 증가: \(newCount)건")
    }

    // MARK: - Save Count

    var currentSaveCount: Int {
        return UserDefaults.standard.integer(forKey: saveCountKey)
    }

    func incrementSaveCount() {
        let newCount = currentSaveCount + 1
        UserDefaults.standard.set(newCount, forKey: saveCountKey)
        print("📊 저장 횟수 증가: \(newCount)건")
    }

    // MARK: - Ad Conditions

    /// 스캔 10회 이상 + 비프리미엄 → 프리뷰 배너 광고
    var shouldShowBannerAd: Bool {
        guard !PremiumManager.shared.isPremium else { return false }
        return currentCount >= adThreshold
    }

    /// 저장 5회 이후 매번 + 비프리미엄 + 마지막 보상형으로부터 10분 경과
    var shouldShowRewardedAd: Bool {
        guard !PremiumManager.shared.isPremium else { return false }
        guard currentSaveCount >= rewardedSaveThreshold else { return false }
        if let last = lastRewardedTime {
            return Date().timeIntervalSince(last) >= rewardedCooldown
        }
        return true
    }

    /// 보상형 광고 완료 시 호출
    func markRewardedShown() {
        lastRewardedTime = Date()
    }

    /// 스캔 횟수 리셋 (디버깅용)
    func reset() {
        UserDefaults.standard.removeObject(forKey: scanCountKey)
        UserDefaults.standard.removeObject(forKey: saveCountKey)
        lastRewardedTime = nil
        print("🔄 스캔/저장 횟수 리셋")
    }
}
