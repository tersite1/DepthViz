//
//  BuildingInfoView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/09.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class BuildingInfoView: UIView {
    private let sectionLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 16, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.text = "Building"
        return label
    }()
    private let backgroundView: UIView = {
        let view = UIView()
        view.translatesAutoresizingMaskIntoConstraints = false
        view.layer.cornerRadius = 16
        view.layer.cornerCurve = .continuous
        view.backgroundColor = UIColor(named: "cellBackgroundColor")
        return view
    }()
    private let buildingIcon: UIImageView = {
        let imageView = UIImageView()
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.image = UIImage(named: "buildingcircleIcon")
        imageView.contentMode = .scaleAspectFit
        NSLayoutConstraint.activate([
            imageView.widthAnchor.constraint(equalToConstant: 36),
            imageView.heightAnchor.constraint(equalToConstant: 36)
        ])
        return imageView
    }()
    private let buildingNameLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 18, weight: .bold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let roadAddressLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let addressLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .regular)
        label.textColor = .secondaryLabel
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let tagView = LidarDetailTagView(tag: .kakaoMap)
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.addSubview(self.sectionLabel)
        NSLayoutConstraint.activate([
            self.sectionLabel.topAnchor.constraint(equalTo: self.topAnchor),
            self.sectionLabel.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 16)
        ])
        
        self.addSubview(self.backgroundView)
        NSLayoutConstraint.activate([
            self.backgroundView.topAnchor.constraint(equalTo: self.sectionLabel.bottomAnchor, constant: 4),
            self.backgroundView.leadingAnchor.constraint(equalTo: self.leadingAnchor),
            self.backgroundView.trailingAnchor.constraint(equalTo: self.trailingAnchor),
        ])
        
        self.backgroundView.addSubview(self.buildingIcon)
        NSLayoutConstraint.activate([
            self.buildingIcon.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.buildingIcon.leadingAnchor.constraint(equalTo: self.backgroundView.leadingAnchor, constant: 16)
        ])
        
        self.backgroundView.addSubview(self.buildingNameLabel)
        NSLayoutConstraint.activate([
            self.buildingNameLabel.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.buildingNameLabel.leadingAnchor.constraint(equalTo: self.buildingIcon.trailingAnchor, constant: 16),
            self.buildingNameLabel.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16)
        ])
        
        self.backgroundView.addSubview(self.roadAddressLabel)
        NSLayoutConstraint.activate([
            self.roadAddressLabel.topAnchor.constraint(equalTo: self.buildingNameLabel.bottomAnchor, constant: 4),
            self.roadAddressLabel.leadingAnchor.constraint(equalTo: self.buildingNameLabel.leadingAnchor)
        ])
        
        self.backgroundView.addSubview(self.addressLabel)
        NSLayoutConstraint.activate([
            self.addressLabel.topAnchor.constraint(equalTo: self.roadAddressLabel.bottomAnchor),
            self.addressLabel.leadingAnchor.constraint(equalTo: self.buildingNameLabel.leadingAnchor)
        ])
        
        self.backgroundView.addSubview(self.tagView)
        NSLayoutConstraint.activate([
            self.tagView.centerYAnchor.constraint(equalTo: self.addressLabel.centerYAnchor),
            self.tagView.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16),
            self.tagView.bottomAnchor.constraint(equalTo: self.backgroundView.bottomAnchor, constant: -12)
        ])
    }
    
    func configure(info: BuildingInfo) {
        self.buildingNameLabel.text = info.placeName
        self.roadAddressLabel.text = info.roadAddressName
        self.addressLabel.text = info.addressName
    }
}
