import UIKit

/// 업데이트 안내 체커
/// GitHub Pages에 호스팅된 JSON으로 최신 버전 확인
/// 닫기 가능, 하루 1회만 표시
///
/// JSON 형식 (https://tersite1.github.io/depthviz/version.json):
/// {
///   "minimum_version": "2.4.0",
///   "app_store_url": "https://apps.apple.com/app/idXXXXXXXXXX"
/// }
final class ForceUpdateChecker {

    static let shared = ForceUpdateChecker()
    private init() {}

    private let versionURL = "https://tersite1.github.io/depthviz/version.json"
    private let lastCheckKey = "update_checker_last_shown_date"

    /// 앱 시작 시 호출 — 구버전이면 업데이트 안내 (하루 1회)
    func checkIfNeeded(from viewController: UIViewController) {
        // 오늘 이미 표시했으면 스킵
        let today = Calendar.current.startOfDay(for: Date())
        if let lastShown = UserDefaults.standard.object(forKey: lastCheckKey) as? Date,
           Calendar.current.isDate(lastShown, inSameDayAs: today) {
            return
        }

        guard let url = URL(string: versionURL) else { return }

        let task = URLSession.shared.dataTask(with: url) { [weak self] data, _, error in
            guard let self = self,
                  error == nil,
                  let data = data,
                  let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let minVersion = json["minimum_version"] as? String,
                  let storeURL = json["app_store_url"] as? String else {
                return
            }

            let currentVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "0.0.0"

            if Self.compareVersions(currentVersion, isLessThan: minVersion) {
                DispatchQueue.main.async {
                    self.showUpdateAlert(from: viewController, storeURL: storeURL)
                    UserDefaults.standard.set(today, forKey: self.lastCheckKey)
                }
            }
        }
        task.resume()
    }

    /// 버전 비교: a < b → true
    private static func compareVersions(_ a: String, isLessThan b: String) -> Bool {
        let aParts = a.split(separator: ".").compactMap { Int($0) }
        let bParts = b.split(separator: ".").compactMap { Int($0) }
        let count = max(aParts.count, bParts.count)

        for i in 0..<count {
            let aVal = i < aParts.count ? aParts[i] : 0
            let bVal = i < bParts.count ? bParts[i] : 0
            if aVal < bVal { return true }
            if aVal > bVal { return false }
        }
        return false
    }

    private func showUpdateAlert(from viewController: UIViewController, storeURL: String) {
        let alert = UIAlertController(
            title: "업데이트 안내",
            message: "새로운 버전이 출시되었습니다. 업데이트하시겠습니까?",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "업데이트", style: .default) { _ in
            if let url = URL(string: storeURL) {
                UIApplication.shared.open(url)
            }
        })
        alert.addAction(UIAlertAction(title: "나중에", style: .cancel))
        viewController.present(alert, animated: true)
    }
}
