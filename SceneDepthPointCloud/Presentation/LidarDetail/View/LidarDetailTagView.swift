//
//  LidarDetailTagView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class LidarDetailTagView: UIView {
    enum Tag {
        case kakaoMap
        case apple
        case pcd
    }
    private let label: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .regular)
        label.textAlignment = .center
        return label
    }()
    
    convenience init(tag: Tag) {
        self.init(frame: CGRect())
        self.configure(tag: tag)
    }
    
    private func configure(tag: Tag) {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.layer.cornerRadius = 4
        self.layer.borderWidth = 1
        
        self.addSubview(self.label)
        NSLayoutConstraint.activate([
            self.label.topAnchor.constraint(equalTo: self.topAnchor, constant: 1),
            self.label.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 4),
            self.label.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -4),
            self.label.bottomAnchor.constraint(equalTo: self.bottomAnchor, constant: -1)
        ])
        
        switch tag {
        case .kakaoMap:
            self.label.text = "Kakao Map"
            self.label.textColor = UIColor(named: "kakaoYellowColor")
            self.layer.borderColor = UIColor(named: "kakaoYellowColor")?.cgColor
        case .apple:
            self.label.text = "Apple"
            self.label.textColor = UIColor(named: "mainColor")
            self.layer.borderColor = UIColor(named: "mainColor")?.cgColor
        case .pcd:
            self.label.text = "PCD"
            self.label.textColor = UIColor(named: "mainColor")
            self.layer.borderColor = UIColor(named: "mainColor")?.cgColor
        }
    }
}
