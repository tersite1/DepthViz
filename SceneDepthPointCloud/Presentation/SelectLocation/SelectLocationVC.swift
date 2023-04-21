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
    /// MainVC 의 함수를 사용하기 위한 delegate
    private weak var delegate: SelectLocationDelegate?
    /// 화면 상단 타이틀 텍스트
    private let titleLabel = SelectLocationTitleLabel()
    /// 측정위치 선택 취소 및 창닫기 버튼
    private let cancelButton = CancelButton()
    /// 현재위치 기준 주소표시 텍스트
    private let currentLocationLabel = RoadAddressLabel()
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
    func configureDelegate(_ delegate: SelectLocationDelegate) {
        self.delegate = delegate
    }
    
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
        
        // cancelButton
        self.cancelButton.addAction(UIAction(handler: { [weak self] _ in
            self?.dismiss(animated: true)
            self?.delegate?.uploadCancel()
        }), for: .touchUpInside)
        self.view.addSubview(self.cancelButton)
        NSLayoutConstraint.activate([
            self.cancelButton.centerYAnchor.constraint(equalTo: self.titleLabel.centerYAnchor),
            self.cancelButton.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -20)
        ])
        
        // currentLocationLabel
        self.view.addSubview(self.currentLocationLabel)
        NSLayoutConstraint.activate([
            self.currentLocationLabel.topAnchor.constraint(equalTo: self.titleLabel.bottomAnchor, constant: 14),
            self.currentLocationLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor)
        ])
    }
}
