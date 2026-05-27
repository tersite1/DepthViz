import Foundation
import UIKit
import Combine
import CommonCrypto

/// 프리미엄 상태 및 기능 토글 관리
/// - 프리미엄 = Buy Me a Coffee 후원 후 코드 입력
/// - 프리미엄 기능: IMU, Odometry, 동영상, 프리미엄 아이콘, 광고 제거
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
        self.isPremium = UserDefaults.standard.bool(forKey: "is_premium")
        self.showOdometry = UserDefaults.standard.bool(forKey: "premium_show_odometry")
        self.showIMUData = UserDefaults.standard.bool(forKey: "premium_show_imu")
        self.saveVideo = UserDefaults.standard.bool(forKey: "premium_save_video")
        self.usePremiumIcon = UserDefaults.standard.bool(forKey: "premium_use_icon")
    }

    // MARK: - Code Validation

    /// 코드의 SHA-256 해시 (바이너리에 평문 노출 방지)
    private static let validCodeHash = "7fb71aa7f7d9b2887158b62d4cb3b28dd840dd1412b3095e5f9da9f34f52e4fa"

    /// 프리미엄 코드 검증 후 활성화
    func validateAndApplyCode(_ code: String) -> Bool {
        let trimmed = code.trimmingCharacters(in: .whitespacesAndNewlines).uppercased()
        guard !trimmed.isEmpty else { return false }

        let hash = Self.sha256(trimmed)
        if hash == Self.validCodeHash {
            DispatchQueue.main.async {
                self.isPremium = true
                self.showOdometry = true
                self.showIMUData = true
                self.saveVideo = true
            }
            UserDefaults.standard.set(trimmed, forKey: "premium_code")
            return true
        }
        return false
    }

    private static func sha256(_ string: String) -> String {
        let data = string.data(using: .utf8)!
        var hash = [UInt8](repeating: 0, count: Int(CC_SHA256_DIGEST_LENGTH))
        data.withUnsafeBytes { bytes in
            _ = CC_SHA256(bytes.baseAddress, CC_LONG(data.count), &hash)
        }
        return hash.map { String(format: "%02x", $0) }.joined()
    }

    // MARK: - Premium Icon

    func setPremiumIcon(_ on: Bool) {
        usePremiumIcon = on
        let iconName: String? = on ? "PremiumIcon" : nil
        guard UIApplication.shared.supportsAlternateIcons else { return }
        UIApplication.shared.setAlternateIconName(iconName) { error in
            if let error = error {
                print("App icon change failed: \(error.localizedDescription)")
            }
        }
    }

    // MARK: - Legacy stubs
    func endPremiumRecording(success: Bool, finalMapFileName: String) {}
    func currentTrajectoryPoints() -> [SIMD3<Float>] { [] }
    func latestTrajectoryPoints() -> [SIMD3<Float>] { [] }
}
