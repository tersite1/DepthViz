//
//  LidarNameSettingView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/06/02.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// 업로드될 Lidar 파일명 설정 커스텀뷰
final class LidarNameSettingView: UIView {
    private let titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 22, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "LiDAR 파일명을 입력하세요"
        return label
    }()
    let lidarNameField: UITextField = {
        let textField = UITextField()
        textField.translatesAutoresizingMaskIntoConstraints = false
        textField.backgroundColor = .clear
        textField.font = UIFont.systemFont(ofSize: 17, weight: .medium)
        textField.textAlignment = .center
        textField.textColor = UIColor(named: "mainColor")
        textField.keyboardType = .default
        textField.borderStyle = .roundedRect
        textField.returnKeyType = .done
        
        NSLayoutConstraint.activate([
            textField.widthAnchor.constraint(equalToConstant: 180),
            textField.heightAnchor.constraint(equalToConstant: 30)
        ])
        return textField
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = .clear
        
        self.addSubview(self.titleLabel)
        NSLayoutConstraint.activate([
            self.titleLabel.topAnchor.constraint(equalTo: self.topAnchor),
            self.titleLabel.centerXAnchor.constraint(equalTo: self.centerXAnchor)
        ])
        
        let frameView = UIView()
        frameView.translatesAutoresizingMaskIntoConstraints = false
        frameView.backgroundColor = UIColor(named: "buildingCellBackgroundColor")
        frameView.layer.cornerRadius = 8
        frameView.layer.cornerCurve = .continuous
        
        self.addSubview(frameView)
        NSLayoutConstraint.activate([
            frameView.topAnchor.constraint(equalTo: self.titleLabel.bottomAnchor, constant: 16),
            frameView.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 16),
            frameView.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -16),
            frameView.heightAnchor.constraint(equalToConstant: 60),
            frameView.bottomAnchor.constraint(equalTo: self.bottomAnchor)
        ])
        
        frameView.addSubview(self.lidarNameField)
        NSLayoutConstraint.activate([
            self.lidarNameField.centerXAnchor.constraint(equalTo: frameView.centerXAnchor),
            self.lidarNameField.centerYAnchor.constraint(equalTo: frameView.centerYAnchor)
        ])
    }
}
