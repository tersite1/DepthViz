//
//  LidarInfoCollectionViewCell.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class LidarInfoCollectionViewCell: UICollectionViewCell {
    static let identifier = "LidarInfoCollectionViewCell"
    enum Section: CaseIterable {
        case lidarList
        case buildingList
    }
    
    private var titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 15, weight: .bold)
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private var floorLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 15, weight: .semibold)
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private var subTitleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 11, weight: .regular)
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private var textLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .secondaryLabel
        label.font = UIFont.systemFont(ofSize: 11, weight: .regular)
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
//    private var rightIcon: UIImageView = {
//        let icon = UIImageView()
//        icon.translatesAutoresizingMaskIntoConstraints = false
//        icon.image = UIImage(systemName: "chevron.right")
//        icon.preferredSymbolConfiguration = UIImage.SymbolConfiguration(pointSize: 15, weight: .medium, scale: .default)
//        icon.tintColor = .white
//        icon.contentMode = .scaleAspectFit
//
//        NSLayoutConstraint.activate([
//            icon.widthAnchor.constraint(equalToConstant: 18),
//            icon.heightAnchor.constraint(equalToConstant: 18)
//        ])
//
//        return icon
//    }()
    private let statusLabel = LidarProcessStatusLabel()
    
    required init?(coder: NSCoder) {
        super.init(coder: coder)
        self.configure()
    }
    
    override init(frame: CGRect) {
        super.init(frame: .zero)
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = UIColor(named: "buildingCellBackgroundColor")
        self.layer.cornerRadius = 8
        self.layer.cornerCurve = .continuous
        
//        self.addSubview(self.rightIcon)
//        NSLayoutConstraint.activate([
//            self.rightIcon.centerYAnchor.constraint(equalTo: self.centerYAnchor),
//            self.rightIcon.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -12)
//        ])
        
        self.addSubview(self.titleLabel)
        NSLayoutConstraint.activate([
            self.titleLabel.topAnchor.constraint(equalTo: self.topAnchor, constant: 8),
            self.titleLabel.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 12)
        ])
        
        self.addSubview(self.floorLabel)
        NSLayoutConstraint.activate([
//            self.floorLabel.widthAnchor.constraint(equalToConstant: 40),
            self.floorLabel.centerYAnchor.constraint(equalTo: self.titleLabel.centerYAnchor),
            self.floorLabel.leadingAnchor.constraint(equalTo: self.titleLabel.trailingAnchor, constant: 8),
//            self.floorLabel.trailingAnchor.constraint(greaterThanOrEqualTo: self.rightIcon.leadingAnchor)
        ])
//        self.floorLabel.trailingAnchor.constraint(greaterThanOrEqualTo: self.rightIcon.leadingAnchor).priority = UILayoutPriority(500)
        
        let stackView = UIStackView(arrangedSubviews: [self.subTitleLabel, self.textLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 2
        stackView.alignment = .leading
        
        self.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.topAnchor.constraint(equalTo: self.titleLabel.bottomAnchor, constant: 8),
            stackView.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 12),
            stackView.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -12)
//            stackView.trailingAnchor.constraint(equalTo: self.rightIcon.leadingAnchor, constant: -8)
        ])
        
        self.addSubview(self.statusLabel)
        NSLayoutConstraint.activate([
            self.statusLabel.centerYAnchor.constraint(equalTo: self.titleLabel.centerYAnchor),
            self.statusLabel.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -12)
        ])
    }
    
    func updateCell(info: LidarInfo) {
        let floor = info.floor >= 0 ? "F\(info.floor)" : "B\(-info.floor)"
        
        self.titleLabel.text = info.buildingName
        self.floorLabel.text = floor
        self.subTitleLabel.text = info.roadAddres
        self.textLabel.text = "\(info.createdAt) / \(info.fileSize)"
        self.statusLabel.updateStatus(to: info.isProgramCompleted ? .completed : .processing)
    }
}
