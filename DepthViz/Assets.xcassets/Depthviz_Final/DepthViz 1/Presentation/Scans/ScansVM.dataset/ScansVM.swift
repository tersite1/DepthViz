//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import Foundation
import Combine

final class ScansVM {
    enum ListMode {
        case lidarList
        case buildingList
    }
    @Published private(set) var mode: ListMode = .lidarList
    @Published private(set) var lidarList: Array = []
    @Published private(set) var networkError: (title: String, text: String)?
    private(set) var isLastPage: Bool = false
    private(set) var fetching: Bool = false
    var listCount: Int {
        return self.mode == .lidarList ? self.lidarList.count : 0
    }
    private var page: Int = 1
    
    init() {
        
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
        
    }
}
