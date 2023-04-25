//
//  UIView+Extension.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

extension UIView {
    func fadeOut() {
        UIView.animate(withDuration: 0.2, animations: {
            self.alpha = 0
        }) { _ in
            self.isHidden = true
        }
    }
    
    func fadeIn() {
        self.isHidden = false
        UIView.animate(withDuration: 0.3) {
            self.alpha = 1
        }
    }
    
    func disappear() {
        self.alpha = 0
        self.isHidden = true
    }
    
    func appear() {
        self.isHidden = false
        self.alpha = 1
    }
}
