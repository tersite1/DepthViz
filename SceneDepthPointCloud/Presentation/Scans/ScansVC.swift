//
//  ScansVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import Combine

final class ScansVC: UIViewController {
    static let identifier = "ScansVC"
    private let reloadButton = ReloadButton()
    private var viewModel: ScansVM?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "SCANS"
        self.configureUI()
        self.configureViewModel()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: true)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.navigationController?.setNavigationBarHidden(true, animated: true)
    }
}

extension ScansVC {
    private func configureUI() {
        self.reloadButton.addAction(UIAction(handler: { [weak self] _ in
            self?.viewModel?.reload()
        }), for: .touchUpInside)
        let rightItem = UIBarButtonItem(customView: self.reloadButton)
        self.navigationItem.setRightBarButton(rightItem, animated: true)
    }
    
    private func configureViewModel() {
        let lidarRepository = LidarRepository()
        self.viewModel = ScansVM(lidarRepository: lidarRepository)
    }
}
