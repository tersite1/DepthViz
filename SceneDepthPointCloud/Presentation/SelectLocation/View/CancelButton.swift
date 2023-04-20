//
//  CancelButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// SelectLocation 의 우상단 취소버튼 커스텀뷰
final class CancelButton: UIButton {
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.setImage(UIImage.init(systemName: "xmark"), for: .normal)
        self.tintColor = .white
        
        NSLayoutConstraint.activate([
            self.widthAnchor.constraint(equalToConstant: 18),
            self.heightAnchor.constraint(equalToConstant: 18)
        ])
    }
}
