//
//  BuildingListCollectionViewCell.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// BuildingListView 에서 표시되는 Cell
final class BuildingListCollectionViewCell: UICollectionViewCell {
    static let identifier = "BuildingListCollectionViewCell"
    enum Section: CaseIterable {
        case main
    }
    
    private var buildingNameLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 15, weight: .semibold)
        label.textAlignment = .left
        return label
    }()
    private var roadAddressLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 11, weight: .regular)
        label.textAlignment = .left
        return label
    }()
    private var distanceLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.textColor = .white
        label.font = UIFont.systemFont(ofSize: 15, weight: .semibold)
        label.textAlignment = .right
        return label
    }()
    
    required init?(coder: NSCoder) {
        super.init(coder: coder)
        self.configure()
    }
    
    override init(frame: CGRect) {
        super.init(frame: .zero)
        self.configure()
    }
    
    override func prepareForReuse() {
        self.backgroundColor = UIColor(named: "buildingCellBackgroundColor")
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        
        let stackView = UIStackView(arrangedSubviews: [self.buildingNameLabel, self.roadAddressLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 2
        stackView.alignment = .leading
        
        self.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.topAnchor.constraint(equalTo: self.topAnchor, constant: 8),
            stackView.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 12),
            stackView.bottomAnchor.constraint(equalTo: self.bottomAnchor, constant: -8)
        ])
        
        self.addSubview(self.distanceLabel)
        NSLayoutConstraint.activate([
            self.distanceLabel.widthAnchor.constraint(equalToConstant: 50),
            self.distanceLabel.centerYAnchor.constraint(equalTo: self.buildingNameLabel.centerYAnchor),
            self.distanceLabel.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -12),
            self.distanceLabel.leadingAnchor.constraint(equalTo: stackView.trailingAnchor, constant: 12)
        ])
        
        self.backgroundColor = UIColor(named: "buildingCellBackgroundColor")
        self.layer.cornerRadius = 8
        self.layer.cornerCurve = .continuous
    }
}

// MARK: INPUT
extension BuildingListCollectionViewCell {
    /// cell 내용설정 함수
    func updateCell(info: BuildingInfo, isSelected: Bool) {
        self.buildingNameLabel.text = info.placeName
        self.roadAddressLabel.text = info.roadAddress
        self.distanceLabel.text = "\(info.distance) M"
        
        if isSelected {
            self.backgroundColor = UIColor(named: "mainColor")
        }
    }
}
