//
//  IndoorSettingView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/25.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class IndoorSettingView: UIView {
    private let titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 22, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .center
        label.text = "실내 위치를 설정하세요"
        return label
    }()
    let floorSwitch: UISegmentedControl = {
        let selector = UISegmentedControl(items: ["지하", "지상"])
        selector.translatesAutoresizingMaskIntoConstraints = false
        selector.selectedSegmentTintColor = UIColor(named: "mainColor")
        selector.backgroundColor = UIColor(named: "selectLocationBackground")
        selector.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .normal)
        selector.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .selected)
        return selector
    }()
    let floorField: UITextField = {
        let textField = UITextField()
        textField.translatesAutoresizingMaskIntoConstraints = false
        textField.backgroundColor = .clear
        textField.font = UIFont.systemFont(ofSize: 17, weight: .medium)
        textField.textAlignment = .center
        textField.textColor = UIColor(named: "mainColor")
        textField.keyboardType = .numbersAndPunctuation
        textField.borderStyle = .roundedRect
        textField.returnKeyType = .done
        
        NSLayoutConstraint.activate([
            textField.widthAnchor.constraint(equalToConstant: 52),
            textField.heightAnchor.constraint(equalToConstant: 30)
        ])
        return textField
    }()
    let floorLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 17, weight: .regular)
        label.textColor = .white
        label.textAlignment = .left
        label.text = "층"
        return label
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
        
        let stackView = UIStackView(arrangedSubviews: [self.floorSwitch, self.floorField, self.floorLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .horizontal
        stackView.spacing = 6
        
        frameView.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.centerXAnchor.constraint(equalTo: frameView.centerXAnchor),
            stackView.centerYAnchor.constraint(equalTo: frameView.centerYAnchor)
        ])
    }
}
