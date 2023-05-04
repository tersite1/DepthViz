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
    @Published private(set) var lidarList: [LidarInfo] = []
    private(set) var isLastPage: Bool = false
    private(set) var fetching: Bool = false
    private var page: Int = 1
    
    init() {
        // MARK: 의존성 받는 부분
    }
}

extension ScansVM {
    func nextPageLidarListFetch() {
        guard self.isLastPage == false else { return }
        
        self.page += 1
        self.fetchLidarList()
    }
}

extension ScansVM {
    private func fetchLidarList() {
        
    }
}
