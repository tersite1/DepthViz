//
//  ScansButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class ScansButton: UIButton {
    let button = UIButton()
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = .tintColor
        self.setTitle("SCNAS", for: .normal)
        self.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        self.titleLabel?.textColor = .white
        self.layer.cornerRadius = 12
        self.layer.cornerCurve = .continuous
        
        NSLayoutConstraint.activate([
            self.heightAnchor.constraint(equalToConstant: 36),
            self.widthAnchor.constraint(equalToConstant: 73)
        ])
    }
}
