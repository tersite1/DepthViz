//
//  UIViewController+Extension.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

extension UIViewController {
    func showAlert(title: String, text: String?) {
        let alert = UIAlertController(title: title, message: text, preferredStyle: .alert)
        let ok = UIAlertAction(title: "OK", style: .default)
        alert.addAction(ok)
        self.present(alert, animated: true)
    }
}
