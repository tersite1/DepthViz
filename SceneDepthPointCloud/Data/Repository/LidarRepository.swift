//
//  LidarRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class LidarRepository: LidarRepositoryInterface {
    func fetchLidarList(page: Int, completion: @escaping (Result<(infos: [LidarInfo], isLastPage: Bool), FetchError>) -> Void) {
        let endpoint = LidarApiService()
        
        endpoint.getLidarList(page: page) { result in
            switch result {
            case.success(let lidarInfoDTO):
                let infos = lidarInfoDTO.resultList.map { $0.toDomain() }
                let isLastPage = page == lidarInfoDTO.totalMap.totalCount
                completion(.success((infos: infos, isLastPage: isLastPage)))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
    }
}
