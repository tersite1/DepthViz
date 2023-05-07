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
        layout.minimumLineSpacing = 14
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
        self.listView.register(LidarInfoCollectionViewCell.self, forCellWithReuseIdentifier: LidarInfoCollectionViewCell.identifier)
        self.listView.delegate = self
        self.listView.dataSource = self
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
        guard let viewModel = self.viewModel else { return }
        switch viewModel.mode {
        case .lidarList:
            if let lidarInfo = viewModel.lidarList[safe: indexPath.item] {
                self.moveToLidarDetailVC(info: lidarInfo)
            }
        case .buildingList:
            // MARK: buildingInfo 구현 필요
            return
        }
    }
    
    func scrollViewDidScroll(_ scrollView: UIScrollView) {
        if self.listView.contentOffset.y > (self.listView.contentSize.height - self.listView.bounds.size.height) {
            guard self.viewModel?.fetching == false,
                  self.viewModel?.isLastPage == false else { return }
            
            self.viewModel?.nextPageListFetch()
        }
    }
}

extension ScansVC: UICollectionViewDataSource {
    func collectionView(_ collectionView: UICollectionView, numberOfItemsInSection section: Int) -> Int {
        return self.viewModel?.listCount ?? 0
    }
    
    func collectionView(_ collectionView: UICollectionView, cellForItemAt indexPath: IndexPath) -> UICollectionViewCell {
        guard let cell = collectionView.dequeueReusableCell(withReuseIdentifier: LidarInfoCollectionViewCell.identifier, for: indexPath) as? LidarInfoCollectionViewCell else {
            return .init()
        }
        guard let info = self.viewModel?.lidarList[safe: indexPath.item] else { return cell }
        
        cell.updateCell(info: info)
        return cell
    }
}

extension ScansVC: UICollectionViewDelegateFlowLayout {
    func collectionView(_ collectionView: UICollectionView, layout collectionViewLayout: UICollectionViewLayout, sizeForItemAt indexPath: IndexPath) -> CGSize {
        return CGSize(width: collectionView.frame.width - (16*2), height: 75)
    }
}

extension ScansVC {
    func moveToLidarDetailVC(info: LidarInfo) {
        guard let lidarDetailVC = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: LidarDetailVC.identifier) as? LidarDetailVC else { return }
        lidarDetailVC.configureViewModel(to: LidarDetailVM(lidarInfo: info))
        self.navigationController?.pushViewController(lidarDetailVC, animated: true)
    }
}
