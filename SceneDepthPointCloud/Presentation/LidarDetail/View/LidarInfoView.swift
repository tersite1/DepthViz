//
//  LidarInfoView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/10.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class LidarInfoView: UIView {
    private let sectionLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 16, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.text = "LiDAR"
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
    private let lidarIcon: UIImageView = {
        let imageView = UIImageView()
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.image = UIImage(named: "lidarCircleIcon")
        imageView.contentMode = .scaleAspectFit
        NSLayoutConstraint.activate([
            imageView.widthAnchor.constraint(equalToConstant: 36),
            imageView.heightAnchor.constraint(equalToConstant: 36)
        ])
        return imageView
    }()
    private let pointsLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 18, weight: .bold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let numberFormatter: NumberFormatter = {
        let numberFormatter = NumberFormatter()
        numberFormatter.numberStyle = .decimal
        return numberFormatter
    }()
    private let fileSizeLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let createdDateLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let fileNameLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .regular)
        label.textColor = .secondaryLabel
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let tagView = LidarDetailTagView(tag: .pcd)
    
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
            self.backgroundView.bottomAnchor.constraint(equalTo: self.bottomAnchor)
        ])
        
        self.backgroundView.addSubview(self.lidarIcon)
        NSLayoutConstraint.activate([
            self.lidarIcon.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.lidarIcon.leadingAnchor.constraint(equalTo: self.backgroundView.leadingAnchor, constant: 16)
        ])
        
        self.backgroundView.addSubview(self.pointsLabel)
        NSLayoutConstraint.activate([
            self.pointsLabel.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.pointsLabel.leadingAnchor.constraint(equalTo: self.lidarIcon.trailingAnchor, constant: 16),
            self.pointsLabel.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16)
        ])
        
        let stackView = UIStackView(arrangedSubviews: [self.fileSizeLabel, self.createdDateLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .horizontal
        stackView.spacing = 12
        stackView.alignment = .center
        
        self.backgroundView.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.topAnchor.constraint(equalTo: self.pointsLabel.bottomAnchor, constant: 4),
            stackView.leadingAnchor.constraint(equalTo: self.pointsLabel.leadingAnchor)
        ])
        
        self.backgroundView.addSubview(self.fileNameLabel)
        NSLayoutConstraint.activate([
            self.fileNameLabel.topAnchor.constraint(equalTo: stackView.bottomAnchor),
            self.fileNameLabel.leadingAnchor.constraint(equalTo: self.pointsLabel.leadingAnchor),
            self.fileNameLabel.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -54)
        ])
        
        self.backgroundView.addSubview(self.tagView)
        NSLayoutConstraint.activate([
            self.tagView.centerYAnchor.constraint(equalTo: self.fileNameLabel.centerYAnchor),
            self.tagView.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16),
            self.tagView.bottomAnchor.constraint(equalTo: self.backgroundView.bottomAnchor, constant: -12)
        ])
    }
    
    func configure(info: LidarDetailInfo) {
        self.pointsLabel.text = "\(self.numberFormatter.string(for: info.totalPoints)!) Points"
        self.fileSizeLabel.text = info.fileSize
        self.createdDateLabel.text = info.createdDate
        if info.originFileName.hasSuffix(".pcd") {
            let index = info.originFileName.firstIndex(of: ".") ?? info.originFileName.endIndex
            self.fileNameLabel.text = String(info.originFileName[..<index])
        } else {
            self.fileNameLabel.text = info.originFileName
        }
    }
}
