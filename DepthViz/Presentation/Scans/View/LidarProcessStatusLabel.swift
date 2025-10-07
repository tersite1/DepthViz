//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import UIKit

final class LidarProcessStatusLabel: UIView {
    enum Status {
        case processing
        case completed
    }
    private let label: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .medium)
        label.textAlignment = .center
        return label
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
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
        
        self.updateStatus(to: .processing)
    }
    
    func updateStatus(to status: Status) {
        if status == .processing {
            self.label.text = "처리중"
            self.label.textColor = .secondaryLabel
            self.layer.borderColor = UIColor.secondaryLabel.cgColor
        } else {
            self.label.text = "완료"
            self.label.textColor = .tintColor
            self.layer.borderColor = UIColor.tintColor.cgColor
        }
    }
}
