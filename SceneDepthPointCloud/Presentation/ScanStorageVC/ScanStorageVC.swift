//
//  ScanStorageVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/07/14.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import SwiftUI
import Combine

final class ScanStorageVC: UIViewController {
    private let listener = ScanInfoRowEventListener()
    private var cancellables: [AnyCancellable] = []
    
    init() {
        super.init(nibName: nil, bundle: nil)
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.bind()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: true)
        AppDelegate.shared.shouldSupportAllOrientation = true
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        AppDelegate.shared.shouldSupportAllOrientation = false
    }
}

extension ScanStorageVC {
    private func configureUI() {
        self.title = "ScanList"
        self.view.backgroundColor = .systemBackground
        
        let hostingVC = UIHostingController(rootView: ScanList().environmentObject(self.listener))
        self.addChild(hostingVC)
        hostingVC.didMove(toParent: self)
        
        hostingVC.view.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(hostingVC.view)
        
        NSLayoutConstraint.activate([
            hostingVC.view.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor),
            hostingVC.view.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor),
            hostingVC.view.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor),
            hostingVC.view.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor)
        ])
    }
    
    private func bind() {
        self.listener.$selectedLidarFileName
            .sink { [weak self] fileName in
                guard let fileName = fileName else { return }
                
                self?.showActionSheet(fileName: fileName)
            }
            .store(in: &self.cancellables)
    }
    
    private func showActionSheet(fileName: String) {
        let alert = UIAlertController(title: fileName, message: "", preferredStyle: .actionSheet)
        alert.addAction(UIAlertAction(title: "파일 공유", style: .default, handler: { [weak self] _ in
            self?.shareScanData(fileName: fileName)
        }))
        alert.addAction(UIAlertAction(title: "파일 삭제", style: .destructive, handler: { [weak self] _ in
            ScanStorage.shared.remove(fileName: fileName)
            // list reload 필요
            self?.showAlert(title: "파일삭제 완료", text: "")
        }))
        alert.addAction(UIAlertAction(title: "취소", style: .default))
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            alert.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
        }
        
        self.present(alert, animated: true)
    }
    
    private func shareScanData(fileName: String) {
        let activityViewController = UIActivityViewController(activityItems: [ScanStorage.shared.fileUrl(fileName: fileName)], applicationActivities: nil)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            activityViewController.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
        }
        
        self.present(activityViewController, animated: true)
        
        activityViewController.completionWithItemsHandler = { (activityType: UIActivity.ActivityType?, completed: Bool, arrayReturnedItems: [Any]?, error: Error?) in
            print("share success")
        }
    }
}
