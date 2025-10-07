/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Contains the application's delegate.
*/

import UIKit
import ARKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {
    static var shared: AppDelegate {
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        return appDelegate
    }
    // 화면 회전을 제어할 변수 선언
    var shouldSupportAllOrientation = false

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        Thread.sleep(forTimeInterval: 3.0)
        return true
    }
    
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

