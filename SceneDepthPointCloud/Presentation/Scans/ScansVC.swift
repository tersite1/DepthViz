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
    private let listView: UICollectionView = {
        let layout = UICollectionViewFlowLayout()
        layout.minimumLineSpacing = 10
        layout.scrollDirection = .vertical
        layout.sectionInset = UIEdgeInsets(top: 0, left: 16, bottom: 0, right: 16)
        return UICollectionView(frame: .zero, collectionViewLayout: layout)
    }()
    private var viewModel: ScansVM?
    private var cancellables: Set<AnyCancellable> = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.title = "SCANS"
        self.configureUI()
        self.configureViewModel()
        self.configureListView()
        self.bindViewModel()
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
        // reloadButton
        self.reloadButton.addAction(UIAction(handler: { [weak self] _ in
            self?.viewModel?.reload()
        }), for: .touchUpInside)
        let rightItem = UIBarButtonItem(customView: self.reloadButton)
        self.navigationItem.setRightBarButton(rightItem, animated: true)
        
        // listView
        self.listView.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(self.listView)
        NSLayoutConstraint.activate([
            self.listView.topAnchor.constraint(equalTo: self.view.topAnchor),
            self.listView.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            self.listView.trailingAnchor.constraint(equalTo: self.view.trailingAnchor),
            self.listView.bottomAnchor.constraint(equalTo: self.view.bottomAnchor)
        ])
    }
    
    private func configureViewModel() {
        let lidarRepository = LidarRepository()
        self.viewModel = ScansVM(lidarRepository: lidarRepository)
    }
    
    private func configureListView() {
        self.listView.delegate = self
    }
}

extension ScansVC {
    private func bindViewModel() {
        self.bindLidarList()
    }
    
    private func bindLidarList() {
        self.viewModel?.$lidarList
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] list in
                self?.listView.reloadData()
            })
            .store(in: &self.cancellables)
    }
}

extension ScansVC: UICollectionViewDelegate {
    func collectionView(_ collectionView: UICollectionView, didSelectItemAt indexPath: IndexPath) {
        print("select")
    }
}

extension ScansVC: UICollectionViewDataSource {
    func collectionView(_ collectionView: UICollectionView, numberOfItemsInSection section: Int) -> Int {
        return self.viewModel?.listCount ?? 0
    }
    
    func collectionView(_ collectionView: UICollectionView, cellForItemAt indexPath: IndexPath) -> UICollectionViewCell {
        return .init()
    }
}

extension ScansVC: UICollectionViewDelegateFlowLayout {
    
}
