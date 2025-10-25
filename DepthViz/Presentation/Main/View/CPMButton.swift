//
//  CPMButton.swift
//  DepthViz
//
//  Created by HYUNAHKO on 9/8/24.
//  Copyright © 2024 Apple. All rights reserved.
//

import UIKit

final class CPMButton: UIButton {
    let button = UIButton()
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }

    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.backgroundColor = .systemBlue
        self.setTitle("CPM", for: .normal)
        self.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        self.titleLabel?.textColor = .white
        self.layer.cornerRadius = 12
        self.layer.cornerCurve = .continuous

        NSLayoutConstraint.activate([
            self.heightAnchor.constraint(equalToConstant: 36),
            self.widthAnchor.constraint(equalToConstant: 73)
        ])
    }
}

