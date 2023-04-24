//
//  BackButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// SelectLocationVC의 좌상단 뒤로가기 커스텀뷰
final class BackButton: UIButton {
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.setImage(UIImage(systemName: "chevron.left"), for: .normal)
        self.setPreferredSymbolConfiguration(UIImage.SymbolConfiguration(pointSize: 18, weight: .regular, scale: .large), forImageIn: .normal)
        self.tintColor = .white
        
        NSLayoutConstraint.activate([
            self.widthAnchor.constraint(equalToConstant: 22),
            self.heightAnchor.constraint(equalToConstant: 22)
        ])
    }
}
