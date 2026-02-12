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

        // 프리미엄 사용자: 불 켜진 건물 런치 오버레이
        if PremiumManager.shared.isPremium {
            showPremiumLaunchOverlay()
        }

        // AppOpenAdManager 델리게이트 설정
        // TODO: AdMob API key를 등록한 후 주석 해제하세요
        /*
        #if canImport(GoogleMobileAds)
        AppOpenAdManager.shared.delegate = self
        #endif
        */
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

// MARK: - Premium Launch Overlay
extension SceneDelegate {
    private func showPremiumLaunchOverlay() {
        guard let window = self.window else { return }

        let overlay = UIView(frame: window.bounds)
        overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        overlay.backgroundColor = UIColor(red: 0.125, green: 0.125, blue: 0.125, alpha: 1)

        // 불 켜진 건물 이미지
        let buildingView = UIImageView(image: UIImage(named: "building4_premium"))
        buildingView.contentMode = .scaleAspectFit
        buildingView.translatesAutoresizingMaskIntoConstraints = false
        overlay.addSubview(buildingView)

        // "Powered by" 라벨
        let poweredLabel = UILabel()
        poweredLabel.text = "Powered by"
        poweredLabel.font = .systemFont(ofSize: 15, weight: .heavy)
        poweredLabel.textColor = .white
        poweredLabel.textAlignment = .center
        poweredLabel.translatesAutoresizingMaskIntoConstraints = false
        overlay.addSubview(poweredLabel)

        // 로고 이미지
        let logoView = UIImageView(image: UIImage(named: "Image 2"))
        logoView.contentMode = .scaleAspectFit
        logoView.translatesAutoresizingMaskIntoConstraints = false
        overlay.addSubview(logoView)

        NSLayoutConstraint.activate([
            buildingView.centerXAnchor.constraint(equalTo: overlay.centerXAnchor),
            buildingView.centerYAnchor.constraint(equalTo: overlay.centerYAnchor, constant: -90),
            buildingView.widthAnchor.constraint(equalToConstant: 320),
            buildingView.heightAnchor.constraint(equalToConstant: 218),

            poweredLabel.centerXAnchor.constraint(equalTo: overlay.centerXAnchor),
            poweredLabel.bottomAnchor.constraint(equalTo: overlay.safeAreaLayoutGuide.bottomAnchor, constant: -128),

            logoView.centerXAnchor.constraint(equalTo: overlay.centerXAnchor),
            logoView.topAnchor.constraint(equalTo: poweredLabel.bottomAnchor, constant: 8),
            logoView.widthAnchor.constraint(equalToConstant: 171),
            logoView.heightAnchor.constraint(equalToConstant: 100),
        ])

        window.addSubview(overlay)

        // 페이드아웃
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.8) {
            UIView.animate(withDuration: 0.4, animations: {
                overlay.alpha = 0
            }) { _ in
                overlay.removeFromSuperview()
            }
        }
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
