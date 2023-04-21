//
//  SelectLocationLargeButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class SelectLocationLargeButton: UIButton {
    enum Status {
        case selectable
        case beforeSetting
        case uploadable
    }
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.layer.cornerRadius = 16
        self.layer.cornerCurve = .continuous
        self.backgroundColor = UIColor(named: "mainColor")
        self.titleLabel?.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        self.titleLabel?.textColor = .white
        
        self.changeStatus(to: .selectable)
    }
    
    func changeStatus(to status: Status) {
        switch status {
        case .selectable:
            self.setTitle("선택 완료", for: .normal)
            self.isUserInteractionEnabled = true
        case .beforeSetting:
            self.setTitle("데이터 업로드", for: .normal)
            self.isUserInteractionEnabled = false
        case .uploadable:
            self.setTitle("데이터 업로드", for: .normal)
            self.isUserInteractionEnabled = true
        }
    }
}
