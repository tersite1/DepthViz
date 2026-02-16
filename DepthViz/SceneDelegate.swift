/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Contains the Scene delegate.
*/

import UIKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

class SceneDelegate: UIResponder, UIWindowSceneDelegate {
    var window: UIWindow?
    
    // 앱 오프닝 광고 표시 여부 (콜드 스타트 시 한 번만 표시)
    private var hasShownAppOpenAd = false

    func scene(_ scene: UIScene, willConnectTo session: UISceneSession, options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = scene as? UIWindowScene else { return }

        self.window = UIWindow(windowScene: windowScene)

        // 로그인 화면 제거 - 직접 MainVC로 이동
        let mainVC = MainVC()
        self.window?.rootViewController = UINavigationController(rootViewController: mainVC)
        self.window?.makeKeyAndVisible()

        // 업데이트 확인
        checkForAppUpdate()
    }
    
    func sceneDidDisconnect(_ scene: UIScene) {
            // Called as the scene is being released by the system.
            // This occurs shortly after the scene enters the background, or when its session is discarded.
            // Release any resources associated with this scene that can be re-created the next time the scene connects.
            // The scene may re-connect later, as its session was not necessarily discarded (see `application:didDiscardSceneSessions` instead).
    }

    func sceneDidBecomeActive(_ scene: UIScene) {
        // 앱이 포그라운드로 올 때 앱 오프닝 광고 표시
        // TODO: AdMob API key를 등록한 후 주석 해제하세요
        /*
        #if canImport(GoogleMobileAds)
        // 콜드 스타트 시 한 번만 표시하도록 제한
        if !hasShownAppOpenAd {
            hasShownAppOpenAd = true
            AppOpenAdManager.shared.showAdIfAvailable()
        }
        #endif
        */
    }

    func sceneWillResignActive(_ scene: UIScene) {
        // Called when the scene will move from an active state to an inactive state.
        // This may occur due to temporary interruptions (ex. an incoming phone call).
        
        // 앱이 백그라운드로 갈 때 다음 포그라운드 진입 시 광고 표시 준비
        hasShownAppOpenAd = false
    }

    func sceneWillEnterForeground(_ scene: UIScene) {
        // Called as the scene transitions from the background to the foreground.
        // Use this method to undo the changes made on entering the background.
    }

    func sceneDidEnterBackground(_ scene: UIScene) {
        // Called as the scene transitions from the foreground to the background.
        // Use this method to save data, release shared resources, and store enough scene-specific state information
        // to restore the scene back to its current state.
    }
}

// MARK: - App Update Check
extension SceneDelegate {
    private func checkForAppUpdate() {
        guard let bundleId = Bundle.main.bundleIdentifier,
              let currentVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String,
              let url = URL(string: "https://itunes.apple.com/lookup?bundleId=\(bundleId)&country=kr") else { return }

        URLSession.shared.dataTask(with: url) { data, _, error in
            guard error == nil,
                  let data = data,
                  let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let results = json["results"] as? [[String: Any]],
                  let storeInfo = results.first,
                  let storeVersion = storeInfo["version"] as? String else { return }

            // 현재 버전이 스토어 버전보다 낮으면 업데이트 권장
            if currentVersion.compare(storeVersion, options: .numeric) == .orderedAscending {
                DispatchQueue.main.async { [weak self] in
                    self?.showUpdateAlert(storeVersion: storeVersion)
                }
            }
        }.resume()
    }

    private func showUpdateAlert(storeVersion: String) {
        guard let rootVC = window?.rootViewController else { return }

        // 이미 alert이 표시 중이면 무시
        if rootVC.presentedViewController is UIAlertController { return }

        let alert = UIAlertController(
            title: "새 버전 출시 (v\(storeVersion))",
            message: "더 나은 스캔 품질과 새로운 기능이 추가되었습니다.\n최신 버전으로 업데이트해주세요.",
            preferredStyle: .alert
        )

        alert.addAction(UIAlertAction(title: "업데이트", style: .default) { _ in
            if let bundleId = Bundle.main.bundleIdentifier,
               let url = URL(string: "https://apps.apple.com/app/id\(bundleId)") {
                // App Store 앱 ID를 사용하는 게 정확하지만, bundleId로도 리다이렉트됨
                UIApplication.shared.open(url)
            }
        })

        alert.addAction(UIAlertAction(title: "나중에", style: .cancel))

        rootVC.present(alert, animated: true)
    }
}

// MARK: - AppOpenAdManagerDelegate
#if canImport(GoogleMobileAds)
extension SceneDelegate: AppOpenAdManagerDelegate {
    func appOpenAdManagerAdDidComplete(_ appOpenAdManager: AppOpenAdManager) {
        // 광고가 완료되면 호출됨 (닫힘 또는 표시 실패)
        print("✅ SceneDelegate: 앱 오프닝 광고 완료")
    }
}
#endif
