//
//  ScansVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class ScansVC: UIViewController {
    static let identifier = "ScansVC"
    let reloadButton = ReloadButton()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "SCANS"
        self.configureUI()
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
            print("reload")
        }), for: .touchUpInside)
        let rightItem = UIBarButtonItem(customView: self.reloadButton)
        self.navigationItem.setRightBarButton(rightItem, animated: true)
    }
}
