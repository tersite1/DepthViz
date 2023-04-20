//
//  SelectLocationVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/20.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import Combine

/// 측정위치 선택화면의 UI 및 UX 담당
final class SelectLocationVC: UIViewController {
    static let identifier = "SelectLocationVC"
    /// 화면 상단 타이틀 텍스트
    private let titleLabel = SelectLocationTitleLabel()
    /// 측정위치 선택화면 관련된 로직담당 객체
    private var viewModel: SelectLocationVM!
    
    /// SelectLocationVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
    }
}

// MARK: INPUT
extension SelectLocationVC {
    func configureViewModel(_ viewModel: SelectLocationVM) {
        self.viewModel = viewModel
    }
}

// MARK: Configure
extension SelectLocationVC {
    /// SelectLocationVC 표시할 UI 설정
    private func configureUI() {
        // titleLabel
        self.view.addSubview(self.titleLabel)
        NSLayoutConstraint.activate([
            self.titleLabel.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 20),
            self.titleLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor)
        ])
    }
}
