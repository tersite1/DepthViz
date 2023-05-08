//
//  BuildingInMapRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// 건물 정보 Repository
final class BuildingInMapRepository: BuildingRepositoryInterface {
    /// BuildingApiService로부터 BuildingNearByGpsDTO를 받아 [BuildingInfo]를 반환하는 함수
    func fetchBuildingInfo(from location: LocationData, page: Int, completion: @escaping (Result<(infos: [BuildingOfMapInfo], isLastPage: Bool), FetchError>) -> Void) {
        let endpoint = KakaoApiService()
        
        let x = Double(location.longitude)
        let y = Double(location.latitude)
        let page = page
        
        endpoint.getSearchByCategory(x: x, y: y, page: page) { result in
            switch result {
            case .success(let buildingDTO):
                let infos = buildingDTO.documents.map { $0.toDomain() }
                let isLastPage = buildingDTO.meta.isEnd
                completion(.success((infos: infos, isLastPage: isLastPage)))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
        
    }
}
