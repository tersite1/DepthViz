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
    private let gpsInfoView = GPSInfoView()
    private let lidarInfoView = LidarInfoView()
    private let openWebButton = OpenWebButton()
    private var viewModel: LidarDetailVM?
    private var cancellables: Set<AnyCancellable> = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "Lidar Detail"
        self.configureUI()
        self.configureBuildingInfo()
        self.bindViewModel()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.viewModel?.fetchDetailInfos()
        AppDelegate.shared.shouldSupportAllOrientation = true
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        AppDelegate.shared.shouldSupportAllOrientation = false
    }
}

extension LidarDetailVC {
    private func configureUI() {
        // buildingInfoView
        self.view.addSubview(self.buildingInfoView)
        NSLayoutConstraint.activate([
            self.buildingInfoView.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 8),
            self.buildingInfoView.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor, constant: 16),
            self.buildingInfoView.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor, constant: -16)
        ])
        
        self.buildingInfoView.disappear()
        
        // gpsInfoView
        self.view.addSubview(self.gpsInfoView)
        NSLayoutConstraint.activate([
            self.gpsInfoView.topAnchor.constraint(equalTo: self.buildingInfoView.bottomAnchor, constant: 32),
            self.gpsInfoView.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor, constant: 16),
            self.gpsInfoView.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor, constant: -16)
        ])
        
        self.gpsInfoView.disappear()
        
        // lidarInfoView
        self.view.addSubview(self.lidarInfoView)
        NSLayoutConstraint.activate([
            self.lidarInfoView.topAnchor.constraint(equalTo: self.gpsInfoView.bottomAnchor, constant: 32),
            self.lidarInfoView.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor, constant: 16),
            self.lidarInfoView.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor, constant: -16)
        ])
        
        self.lidarInfoView.disappear()
        
        // openWebButton
        self.view.addSubview(self.openWebButton)
        NSLayoutConstraint.activate([
            self.openWebButton.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: 0),
            self.openWebButton.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor, constant: 16),
            self.openWebButton.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor, constant: -16),
            self.openWebButton.topAnchor.constraint(equalTo: self.lidarInfoView.bottomAnchor, constant: 32)
        ])
        
        self.openWebButton.addAction(UIAction(handler: { [weak self] _ in
            guard let addressId = self?.viewModel?.addressId else { return }
            
            if let url = URL(string: NetworkURL.Web.addressURL(addressId: addressId)) {
                UIApplication.shared.open(url, options: [:])
            }
        }), for: .touchUpInside)
    }
    
    private func configureBuildingInfo() {
        let tapGesture: UITapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(tapBuildingInfo(_:)))
        self.buildingInfoView.addGestureRecognizer(tapGesture)
    }
    
    @objc private func tapBuildingInfo(_ gesture: UITapGestureRecognizer) {
        guard let url = self.viewModel?.buildingInfo?.placeURL else { return }
        self.showWebView(url: url)
    }
    
    private func showWebView(url: String) {
        guard let webViewVC = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: WebViewVC.identifier) as? WebViewVC else { return }
        
        webViewVC.setUrl(to: url)
        self.navigationController?.pushViewController(webViewVC, animated: true)
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
                self?.showGpsInfoView()
                self?.showLidarInfoView()
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
    
    private func showGpsInfoView() {
        guard let gpsInfo = self.viewModel?.lidarDetailInfo else { return }
        
        self.gpsInfoView.configure(info: gpsInfo)
        self.gpsInfoView.fadeIn()
    }
    
    private func showLidarInfoView() {
        guard let lidarInfo = self.viewModel?.lidarDetailInfo else { return }
        
        self.lidarInfoView.configure(info: lidarInfo)
        self.lidarInfoView.fadeIn()
    }
}

extension LidarDetailVC {
    func configureViewModel(to viewModel: LidarDetailVM) {
        self.viewModel = viewModel
    }
}
