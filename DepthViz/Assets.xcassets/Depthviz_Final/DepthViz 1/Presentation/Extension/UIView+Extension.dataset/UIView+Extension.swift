//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
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
