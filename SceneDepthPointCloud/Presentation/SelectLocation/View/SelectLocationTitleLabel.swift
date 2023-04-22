//
//  SelectLocationTitleLabel.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

/// SelectLocationVC 상단 titleLabel 커스텀뷰
final class SelectLocationTitleLabel: UILabel {
    enum Status {
        case selectLocation
        case selectBuilding
        case selectedBuilding
        case selectCompleted
    }
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.textColor = .white
        self.font = UIFont.systemFont(ofSize: 22, weight: .semibold)
        self.textAlignment = .center
        self.numberOfLines = 1
        self.changeText(to: .selectLocation)
    }
    
    /// Status 값에 따라 text 내용 변경하는 함수
    func changeText(to status: Status) {
        if status == .selectLocation {
            self.text = "측정 위치를 설정하세요"
        } else {
            self.text = "측정 건물을 설정하세요"
        }
    }
}
