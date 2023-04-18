//
//  PointCloudCountView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/18.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class PointCloudCountView: UIView {
    private var icon: UIImageView = {
        let imageView = UIImageView()
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.image = UIImage(named: "dots")
        imageView.contentMode = .scaleAspectFit
        
        NSLayoutConstraint.activate([
            imageView.widthAnchor.constraint(equalToConstant: 18),
            imageView.heightAnchor.constraint(equalToConstant: 18)
        ])
        
        return imageView
    }()
    
    private var countLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        return label
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.layer.cornerRadius = 8
        self.layer.cornerCurve = .continuous
        self.backgroundColor = .black
        
        let stackView = UIStackView(arrangedSubviews: [self.icon, self.countLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.alignment = .center
        stackView.axis = .horizontal
        stackView.spacing = 7
        
        self.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.topAnchor.constraint(equalTo: self.topAnchor, constant: 3),
            stackView.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 5),
            stackView.trailingAnchor.constraint(equalTo: self.trailingAnchor, constant: -5),
            stackView.bottomAnchor.constraint(equalTo: self.bottomAnchor, constant: -3)
        ])
        
        self.updateCount(to: 0)
    }
}

// MARK: INPUT
extension PointCloudCountView {
    func updateCount(to count: Int) {
        self.countLabel.text = "\(count) P"
    }
}
