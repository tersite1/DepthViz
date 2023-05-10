//
//  OpenWebButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/10.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class OpenWebButton: UIButton {
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.layer.cornerRadius = 16
        self.layer.cornerCurve = .continuous
        self.backgroundColor = UIColor(named: "mainColor")
        self.titleLabel?.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        self.titleLabel?.textColor = .white
        self.setTitle("Open in Web", for: .normal)
        
        NSLayoutConstraint.activate([
            self.heightAnchor.constraint(equalToConstant: 50)
        ])
    }
}
