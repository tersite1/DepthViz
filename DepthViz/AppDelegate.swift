/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Contains the application's delegate.
*/

import UIKit
import ARKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {
    static var shared: AppDelegate {
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        return appDelegate
    }
    // 화면 회전을 제어할 변수 선언
    var shouldSupportAllOrientation = false

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // 앱 실행 횟수 증가
        incrementLaunchCount()
        
        // AdMob 초기화 (SDK가 설치된 경우에만)
        #if canImport(GoogleMobileAds)
        initializeAdMob()
        #endif
        
        return true
    }
    
    /// 앱 실행 횟수 증가
    private func incrementLaunchCount() {
        let launchCount = UserDefaults.standard.integer(forKey: "app_launch_count")
        UserDefaults.standard.set(launchCount + 1, forKey: "app_launch_count")
        print("📱 앱 실행 횟수: \(launchCount + 1)")
    }
    
    #if canImport(GoogleMobileAds)
    /// AdMob SDK 초기화
    private func initializeAdMob() {
        GADMobileAds.sharedInstance().start { status in
            print("✅ AdMob SDK initialized successfully")
            // 초기화 상태 출력 (디버깅용)
            for (key, value) in status.adapterStatusesByClassName {
                print("  \(key): \(value.state.rawValue) - \(value.description)")
            }
            
            // 앱 오프닝 광고 미리 로드
            Task {
                await AppOpenAdManager.shared.loadAd()
            }
        }
    }
    #endif
    
    func application(_ application: UIApplication, supportedInterfaceOrientationsFor window: UIWindow?) -> UIInterfaceOrientationMask {
        if shouldSupportAllOrientation {
            return .all
        } else {
            return .portrait
        }
    }
    
    func application(_ application: UIApplication, configurationForConnecting connectingSceneSession: UISceneSession, options: UIScene.ConnectionOptions) -> UISceneConfiguration {
        // Called when a new scene session is being created.
        // Use this method to select a configuration to create the new scene with.
        return UISceneConfiguration(name: "Default Configuration", sessionRole: connectingSceneSession.role)
    }
    
    func application(_ application: UIApplication, didDiscardSceneSessions sceneSessions: Set<UISceneSession>) {
        // Called when the user discards a scene session.
        // If any sessions were discarded while the application was not running, this will be called shortly after application:didFinishLaunchingWithOptions.
        // Use this method to release any resources that were specific to the discarded scenes, as they will not return.
    }
}

