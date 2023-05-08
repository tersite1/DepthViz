//
//  BuildingRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class BuildingRepository: BuildingRepositoryInterface {
    func fetchBuildingInfo(addressId: String, completion: @escaping (Result<BuildingInfo, FetchError>) -> Void) {
        let endpoint = BuildingApiService()
        
        endpoint.getBuildingInfo(addressId: addressId) { result in
            switch result {
            case.success(let buildingDTO):
                completion(.success(buildingDTO.resultObject.toDomain()))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
    }
}
