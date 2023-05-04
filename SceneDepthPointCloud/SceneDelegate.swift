//
//  SceneDelegate.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

class SceneDelegate: UIResponder, UIWindowSceneDelegate {
    var window: UIWindow?
    
    func scene(_ scene: UIScene, willConnectTo session: UISceneSession, options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = scene as? UIWindowScene else { return }
        
        self.window?.windowScene = windowScene
        
        guard let rootVC = UIStoryboard(name: "Main", bundle: nil).instantiateInitialViewController() else { return }
        let navigationVC = UINavigationController(rootViewController: rootVC)
        print("ok")
        self.window?.rootViewController = navigationVC
    }
}
