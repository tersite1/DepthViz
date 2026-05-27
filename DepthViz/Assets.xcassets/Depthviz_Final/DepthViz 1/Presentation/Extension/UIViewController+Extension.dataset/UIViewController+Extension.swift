//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//



import UIKit

extension UIViewController {
    /// title, text를 표시하는 Alert
    func showAlert(title: String, text: String?) {
        let alert = UIAlertController(title: title, message: text, preferredStyle: .alert)
        let ok = UIAlertAction(title: "OK", style: .default)
        alert.addAction(ok)
        self.present(alert, animated: true)
    }
    
    /// 위치 정보가 필요한 경우의 Alert
    func showGPSWarning() {
        let alert = UIAlertController(title: "\"Point Cloud\"의 위치 사용 권한이 필요합니다.", message: "위치 권한을 허용해야만 앱을 사용하실 수 있습니다.", preferredStyle: .alert)
        let setting = UIAlertAction(title: "설정", style: .cancel) { _ in
            if let appSettings = URL(string: UIApplication.openSettingsURLString) {
                UIApplication.shared.open(appSettings,options: [:],completionHandler: nil)
            }
        }
        alert.addAction(setting)
        self.present(alert, animated: true)
    }
}
