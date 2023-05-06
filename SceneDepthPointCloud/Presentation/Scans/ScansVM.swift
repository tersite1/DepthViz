//
//  ScansVM.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import Combine

final class ScansVM {
    enum ListMode {
        case lidarList
        case buildingList
    }
    @Published private(set) var mode: ListMode = .lidarList
    @Published private(set) var lidarList: [LidarInfo] = []
    @Published private(set) var networkError: (title: String, text: String)?
    private(set) var isLastPage: Bool = false
    private(set) var fetching: Bool = false
    var listCount: Int {
        return self.mode == .lidarList ? self.lidarList.count : 0
    }
    private var page: Int = 1
    private let lidarRepository: LidarRepositoryInterface
    
    init(lidarRepository: LidarRepositoryInterface) {
        self.lidarRepository = lidarRepository
        
        self.reload()
    }
}

// MARK: INPUT
extension ScansVM {
    func nextPageListFetch() {
        switch self.mode {
        case .lidarList:
            self.nextPageLidarListFetch()
        case .buildingList:
            // MARK: BuildingList 함수 작성
            return
        }
    }
    
    func reload() {
        switch self.mode {
        case .lidarList:
            self.page = 1
            self.isLastPage = false
            self.lidarList = []
            self.fetchLidarList()
            
        case .buildingList:
            // MARK: BuildingList 수신 API 확인 필요
            return
        }
    }
}

extension ScansVM {
    private func nextPageLidarListFetch() {
        guard self.isLastPage == false else { return }

        self.page += 1
        self.fetchLidarList()
    }
    
    private func fetchLidarList() {
        guard self.fetching == false,
              self.isLastPage == false else { return }
        
        self.fetching = true
        
        self.lidarRepository.fetchLidarList(page: self.page) { [weak self] result in
            switch result {
            case .success((let infos, let isLastPage)):
                self?.lidarList += infos
                self?.isLastPage = isLastPage
                
            case .failure(let fetchError):
                self?.networkError = (title: "Fetch LidarList Error", text: fetchError.message)
            }
            self?.fetching = false
        }
    }
}
