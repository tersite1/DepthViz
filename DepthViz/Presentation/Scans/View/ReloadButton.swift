//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//

import UIKit

final class ReloadButton: UIButton {
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.setImage(UIImage.init(systemName: "arrow.clockwise"), for: .normal)
        self.setPreferredSymbolConfiguration(UIImage.SymbolConfiguration(pointSize: 18, weight: .medium, scale: .default), forImageIn: .normal)
        self.tintColor = .tintColor
        self.contentMode = .scaleAspectFit
    }
}
