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
        
        // AppOpenAdManager 델리게이트 설정
        #if canImport(GoogleMobileAds)
        AppOpenAdManager.shared.delegate = self
        #endif
    }
    
    func sceneDidDisconnect(_ scene: UIScene) {
            // Called as the scene is being released by the system.
            // This occurs shortly after the scene enters the background, or when its session is discarded.
            // Release any resources associated with this scene that can be re-created the next time the scene connects.
            // The scene may re-connect later, as its session was not necessarily discarded (see `application:didDiscardSceneSessions` instead).
    }

    func sceneDidBecomeActive(_ scene: UIScene) {
        // 앱이 포그라운드로 올 때 앱 오프닝 광고 표시
        #if canImport(GoogleMobileAds)
        // 콜드 스타트 시 한 번만 표시하도록 제한
        if !hasShownAppOpenAd {
            hasShownAppOpenAd = true
            AppOpenAdManager.shared.showAdIfAvailable()
        }
        #endif
    }

    func sceneWillResignActive(_ scene: UIScene) {
        // Called when the scene will move from an active state to an inactive state.
        // This may occur due to temporary interruptions (ex. an incoming phone call).
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
    
    func sceneWillResignActive(_ scene: UIScene) {
        // 앱이 백그라운드로 갈 때 다음 포그라운드 진입 시 광고 표시 준비
        hasShownAppOpenAd = false
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
