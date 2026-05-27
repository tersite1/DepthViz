//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
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
        self.bindViewModel()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: true)
        AppDelegate.shared.shouldSupportAllOrientation = true
        self.viewModel?.reload()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        AppDelegate.shared.shouldSupportAllOrientation = false
    }
    
    override func viewWillTransition(to size: CGSize, with coordinator: UIViewControllerTransitionCoordinator) {
        super.viewWillTransition(to: size, with: coordinator)
        self.listView.collectionViewLayout.invalidateLayout()
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
            self.listView.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor),
            self.listView.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor),
            self.listView.bottomAnchor.constraint(equalTo: self.view.bottomAnchor)
        ])
    }
    
    private func configureViewModel() {
        self.viewModel = ScansVM()
    }
}

extension ScansVC {
    private func bindViewModel() {
        self.bindLidarList()
        self.bindNetworkError()
    }
    
    private func bindLidarList() {
        self.viewModel?.$lidarList
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] list in
                self?.listView.reloadData()
            })
            .store(in: &self.cancellables)
    }
    
    private func bindNetworkError() {
        self.viewModel?.$networkError
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] error in
                guard let error = error else { return }
                self?.showAlert(title: error.title, text: error.text)
            })
            .store(in: &self.cancellables)
    }
}

extension ScansVC: UICollectionViewDelegate {
    func collectionView(_ collectionView: UICollectionView, didSelectItemAt indexPath: IndexPath) {
        guard let viewModel = self.viewModel else { return }
        switch viewModel.mode {
        case .lidarList:
            if let lidarInfo = viewModel.lidarList[safe: indexPath.item] {}
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

extension ScansVC: UICollectionViewDelegateFlowLayout {
    func collectionView(_ collectionView: UICollectionView, layout collectionViewLayout: UICollectionViewLayout, sizeForItemAt indexPath: IndexPath) -> CGSize {
        return CGSize(width: collectionView.frame.width - (16*2), height: 75)
    }
}
