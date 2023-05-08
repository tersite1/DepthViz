//
//  SettedInfoView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/26.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// 업로드될 데이터 표시 커스텀뷰
final class SettedInfoView: UIView {
    private let titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 22, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "준비 완료!"
        return label
    }()
    private let placeCell = SettedInfoCell()
    private let lidarCell = SettedInfoCell()
    private let numberFormatter: NumberFormatter = {
        let numberFormatter = NumberFormatter()
        numberFormatter.numberStyle = .decimal
        return numberFormatter
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
        
        self.addSubview(self.placeCell)
        NSLayoutConstraint.activate([
            self.placeCell.topAnchor.constraint(equalTo: self.titleLabel.bottomAnchor, constant: 16),
            self.placeCell.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 16),
            self.placeCell.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -16)
        ])
        
        self.addSubview(self.lidarCell)
        NSLayoutConstraint.activate([
            self.lidarCell.topAnchor.constraint(equalTo: self.placeCell.bottomAnchor, constant: 16),
            self.lidarCell.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 16),
            self.lidarCell.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -16)
        ])
    }
    
    /// SettedInfoView 표시될 내용 설정 함수
    func configureInfos(buildingInfo: BuildingOfMapInfo, floor: Int, lidarData: LiDARData) {
        let placeFloor = floor >= 0 ? "F\(floor)" : "B\(-floor)"
        let placeTitle = "\(buildingInfo.placeName) floor \(placeFloor)"
        self.placeCell.setText(title: placeTitle, text: buildingInfo.roadAddress)
        
        let lidarText = "\(self.numberFormatter.string(for: lidarData.pointCount)!) Points / \(lidarData.lidarFileSize)"
        self.lidarCell.setText(title: lidarData.lidarFileName, text: lidarText)
    }
}

/// SettedInfoView내에서 재사용되는 서브 커스텀뷰
final class SettedInfoCell: UIView {
    private let titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 16, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "title"
        return label
    }()
    private let textLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 13, weight: .regular)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "text"
        return label
    }()
    private lazy var stackView: UIStackView = {
        let stackView = UIStackView(arrangedSubviews: [self.titleLabel, self.textLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 8
        stackView.contentMode = .center
        
        return stackView
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = UIColor(named: "buildingCellBackgroundColor")
        NSLayoutConstraint.activate([
            self.heightAnchor.constraint(equalToConstant: 75)
        ])
        self.layer.cornerRadius = 8
        self.layer.cornerCurve = .continuous
        
        self.addSubview(self.stackView)
        NSLayoutConstraint.activate([
            self.stackView.centerXAnchor.constraint(equalTo: self.centerXAnchor),
            self.stackView.centerYAnchor.constraint(equalTo: self.centerYAnchor)
        ])
    }
    
    /// 표시될 내용 설정 함수
    func setText(title: String, text: String) {
        self.titleLabel.text = title
        self.textLabel.text = text
    }
}
