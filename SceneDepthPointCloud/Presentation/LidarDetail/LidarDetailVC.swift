//
//  LidarDetailVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/07.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import Combine

final class LidarDetailVC: UIViewController {
    static let identifier = "LidarDetailVC"
    private let buildingInfoView = BuildingInfoView()
    private var viewModel: LidarDetailVM?
    private var cancellables: Set<AnyCancellable> = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "Lidar Detail"
        self.configureUI()
        self.bindViewModel()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.viewModel?.fetchDetailInfos()
    }
}

extension LidarDetailVC {
    private func configureUI() {
        self.view.addSubview(self.buildingInfoView)
        NSLayoutConstraint.activate([
            self.buildingInfoView.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 8),
            self.buildingInfoView.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 16),
            self.buildingInfoView.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -16)
        ])
        
        self.buildingInfoView.disappear()
    }
}

extension LidarDetailVC {
    private func bindViewModel() {
        self.bindInfosDownloaded()
    }
    
    private func bindInfosDownloaded() {
        self.viewModel?.$infosDownloaded
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] downloaded in
                guard downloaded else { return }
                
                self?.showBuildingInfoView()
            })
            .store(in: &self.cancellables)
    }
}

extension LidarDetailVC {
    private func showBuildingInfoView() {
        guard let buildingInfo = self.viewModel?.buildingInfo else { return }
        
        self.buildingInfoView.configure(info: buildingInfo)
        self.buildingInfoView.fadeIn()
    }
}

extension LidarDetailVC {
    func configureViewModel(to viewModel: LidarDetailVM) {
        self.viewModel = viewModel
    }
}
