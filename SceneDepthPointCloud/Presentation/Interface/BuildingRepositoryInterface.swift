//
//  BuildingRepositoryInterface.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Data Layer를 간접접근하기 위한 인터페이스
protocol BuildingRepositoryInterface: AnyObject {
    /// LocationData로부터 100m근방 장소들을 받는 함수
    func fetchBuildingInfo(from location: LocationData, page: Int, completion: @escaping (Result<(infos: [BuildingOfMapInfo], isLastPage: Bool), FetchError>) -> Void)
}
