import Foundation
import UIKit
import Combine

/// 프리미엄 상태 및 기능 토글 관리
class PremiumManager: ObservableObject {
    static let shared = PremiumManager()

    @Published var isPremium: Bool {
        didSet { UserDefaults.standard.set(isPremium, forKey: "is_premium") }
    }
    @Published var showOdometry: Bool {
        didSet { UserDefaults.standard.set(showOdometry, forKey: "premium_show_odometry") }
    }
    @Published var showIMUData: Bool {
        didSet { UserDefaults.standard.set(showIMUData, forKey: "premium_show_imu") }
    }
    @Published var saveVideo: Bool {
        didSet { UserDefaults.standard.set(saveVideo, forKey: "premium_save_video") }
    }
    @Published var usePremiumIcon: Bool {
        didSet { UserDefaults.standard.set(usePremiumIcon, forKey: "premium_use_icon") }
    }

    private init() {
        #if DEBUG
        // DEBUG: 프리미엄 기능 전부 활성화 (광고는 ScanCountManager에서 별도 제어)
        UserDefaults.standard.register(defaults: [
            "is_premium": true,
            "premium_show_odometry": true,
            "premium_show_imu": true,
            "premium_save_video": true
        ])
        #endif

        self.isPremium = UserDefaults.standard.bool(forKey: "is_premium")
        self.showOdometry = UserDefaults.standard.bool(forKey: "premium_show_odometry")
        self.showIMUData = UserDefaults.standard.bool(forKey: "premium_show_imu")
        self.saveVideo = UserDefaults.standard.bool(forKey: "premium_save_video")
        self.usePremiumIcon = UserDefaults.standard.bool(forKey: "premium_use_icon")
    }

    func unlock() {
        DispatchQueue.main.async {
            self.isPremium = true
            // 잠금 해제 시 기본값으로 기능 활성화
            self.showOdometry = true
            self.showIMUData = true
            self.saveVideo = true
        }
    }

    func setPremiumIcon(_ on: Bool) {
        usePremiumIcon = on
        let iconName: String? = on ? "PremiumIcon" : nil
        guard UIApplication.shared.supportsAlternateIcons else { return }
        UIApplication.shared.setAlternateIconName(iconName) { error in
            if let error = error {
                print("❌ 앱 아이콘 변경 실패: \(error.localizedDescription)")
            } else {
                print("✅ 앱 아이콘 변경: \(on ? "프리미엄" : "기본")")
            }
        }
    }

    // MARK: - Legacy stubs (다른 코드에서 참조 시 호환성 유지)
    func endPremiumRecording(success: Bool, finalMapFileName: String) {}
    func currentTrajectoryPoints() -> [SIMD3<Float>] { [] }
    func latestTrajectoryPoints() -> [SIMD3<Float>] { [] }
}
