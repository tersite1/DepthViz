//
//  LidarApiService.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class LidarApiService {
    /// ply 데이터를 서버로 전송
    func upload(buildingInfo: BuildingInfo, location: IndoorData, file: LiDARData, completion: @escaping (Result<Bool, FetchError>) -> Void) {
        guard let locationJsonData = try? JSONEncoder().encode(location) else {
            print("Error: encode location")
            completion(.failure(.client))
            return
        }
        guard let addressJsonData = try? JSONEncoder().encode(buildingInfo.dto) else {
            print("Error: encode address")
            completion(.failure(.client))
            return
        }
        
        guard let locationString = String(data: locationJsonData, encoding: .utf8) else {
            print("Error: encode location to string")
            completion(.failure(.client))
            return
        }
        guard let addressString = String(data: addressJsonData, encoding: .utf8) else {
            print("Error: encode address to string")
            completion(.failure(.client))
            return
        }
        
        Network.uploadData(url: NetworkURL.Server.lidars, address: addressString, location: locationString, file: file) { result in
            switch result.status {
            case .SUCCESS:
                guard let data = result.data else {
                    completion(.failure(.empty))
                    return
                }
                
                print(String(data: data, encoding: .utf8)!)
                completion(.success(true))
                
            case .ERROR(let statusCode):
                completion(.failure(.server(statusCode)))
            }
        }
    }
}
