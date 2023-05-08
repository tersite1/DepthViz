//
//  LidarDetailVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/07.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class LidarDetailVC: UIViewController {
    static let identifier = "LidarDetailVC"
    private var viewModel: LidarDetailVM?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "Lidar Detail"
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.viewModel?.fetchDetailInfos()
    }
}

extension LidarDetailVC {
    func configureViewModel(to viewModel: LidarDetailVM) {
        self.viewModel = viewModel
    }
}
