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
        label.textAlignment = .left
        label.text = "Building"
        return label
    }()
    private let backgroundView: UIView = {
        let view = UIView()
        view.translatesAutoresizingMaskIntoConstraints = false
        view.layer.cornerRadius = 16
        view.layer.cornerCurve = .continuous
        view.backgroundColor = UIColor(named: "selectLocationBackground")
        return view
    }()
    
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
            self.backgroundView.heightAnchor.constraint(equalToConstant: 87)
        ])
    }
}
