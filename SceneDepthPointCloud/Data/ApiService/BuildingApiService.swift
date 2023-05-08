//
//  BuildingApiService.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class BuildingApiService {
    func getBuildingInfo(addressId: String, completion: @escaping (Result<BuildingInfoDTO, FetchError>) -> Void) {
        Network.request(url: NetworkURL.Server.buildings + "/\(addressId)", method: .get) { result in
            switch result.status {
            case .SUCCESS:
                guard let data = result.data else {
                    completion(.failure(.empty))
                    return
                }
                
                guard let dto = try? JSONDecoder().decode(BuildingInfoDTO.self, from: data) else {
                    completion(.failure(.decode))
                    return
                }
                
                completion(.success(dto))
                
            case .ERROR(let statusCode):
                completion(.failure(.server(statusCode)))
            }
        }
    }
}
