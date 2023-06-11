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
    private var deleteButton: UIButton = {
        let button = UIButton()
        button.translatesAutoresizingMaskIntoConstraints = false
        button.setImage(UIImage(systemName: "trash"), for: .normal)
        button.setPreferredSymbolConfiguration(UIImage.SymbolConfiguration(pointSize: 18, weight: .medium, scale: .default), forImageIn: .normal)
        button.tintColor = .systemRed
        button.contentMode = .scaleAspectFit
        return button
    }()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "Lidar Detail"
        self.configureUI()
        self.configureBuildingInfo()
        self.configureLidarInfo()
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
        // delete button
        self.deleteButton.addAction(UIAction(handler: { [weak self] _ in
            self?.showAlertForDelete()
        }), for: .touchUpInside)
        let rightItem = UIBarButtonItem(customView: self.deleteButton)
        self.navigationItem.setRightBarButton(rightItem, animated: true)
        
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
            guard let self = self,
                  let addressId = self.viewModel?.addressId else { return }
            
            if let url = URL(string: NetworkURL.Web.addressURL(addressId: addressId)) {
                let activityViewController = UIActivityViewController(activityItems: [url], applicationActivities: nil)
                
                if UIDevice.current.userInterfaceIdiom == .pad {
                    activityViewController.popoverPresentationController?.sourceView = self.openWebButton
                }
                
                self.present(activityViewController, animated: true)
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
    
    private func configureLidarInfo() {
        let tapGesture: UITapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(tapLidarInfo(_:)))
        self.lidarInfoView.addGestureRecognizer(tapGesture)
    }
    
    @objc private func tapLidarInfo(_ gesture: UITapGestureRecognizer) {
        // showAlert
        let alert = UIAlertController(title: "LiDAR 파일을 다운로드하시겠습니까?", message: "받고자 하는 파일 형식을 선택하세요", preferredStyle: .actionSheet)
        let lidar = UIAlertAction(title: "PCD", style: .default) { [weak self] _ in
            self?.viewModel?.downloadPCD()
        }
        let pcd = UIAlertAction(title: "PLY", style: .default) { [weak self] _ in
            self?.viewModel?.downloadPLY()
        }
        let cancel = UIAlertAction(title: "취소", style: .cancel)
        alert.addAction(lidar)
        alert.addAction(pcd)
        alert.addAction(cancel)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            alert.popoverPresentationController?.sourceView = self.lidarInfoView
        }
        
        self.present(alert, animated: true)
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
        self.bindDeleteCompleted()
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
    
    private func bindDeleteCompleted() {
        self.viewModel?.$deleteCompleted
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] deleted in
                guard let deleted = deleted else { return }
                
                if deleted {
                    self?.showAlertAndExit()
                } else {
                    let message = self?.viewModel?.serverError ?? ""
                    self?.showAlert(title: "Delete failed", text: message)
                }
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
        guard let gpsInfo = self.viewModel?.lidarDetailInfo,
              let buildingInfo = self.viewModel?.buildingInfo else { return }
        
        self.gpsInfoView.configure(info: gpsInfo, buildingInfo: buildingInfo)
        self.gpsInfoView.fadeIn()
    }
    
    private func showLidarInfoView() {
        guard let lidarInfo = self.viewModel?.lidarDetailInfo else { return }
        
        self.lidarInfoView.configure(info: lidarInfo)
        self.lidarInfoView.fadeIn()
    }
    
    private func showAlertForDelete() {
        let alert = UIAlertController(title: "LiDAR 파일을 삭제하시겠습니까?", message: "삭제 후 복원되지 않습니다.", preferredStyle: .alert)
        let cancel = UIAlertAction(title: "취소", style: .default)
        let delete = UIAlertAction(title: "삭제", style: .destructive) { [weak self] _ in
            self?.viewModel?.deleteLidar()
        }
        alert.addAction(cancel)
        alert.addAction(delete)
        self.present(alert, animated: true)
    }
    
    private func showAlertAndExit() {
        let alert = UIAlertController(title: "삭제가 완료되었습니다", message: "", preferredStyle: .alert)
        let ok = UIAlertAction(title: "확인", style: .default) { [weak self] _ in
            self?.navigationController?.popViewController(animated: true)
        }
        alert.addAction(ok)
        self.present(alert, animated: true)
    }
}

extension LidarDetailVC {
    func configureViewModel(to viewModel: LidarDetailVM) {
        self.viewModel = viewModel
    }
}
