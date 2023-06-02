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
    func upload(buildingInfo: BuildingOfMapInfo, location: IndoorData, file: LiDARData, handler: @escaping((Double) -> Void), completion: @escaping (Result<Bool, FetchError>) -> Void) {
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
        
        Network.uploadData(url: NetworkURL.Server.lidars, address: addressString, location: locationString, file: file, handler: { progress in
            handler(progress)
        }) { result in
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
    
    func getLidarList(page: Int, completion: @escaping (Result<LidarInfoDTO, FetchError>) -> Void) {
        var parameters: [String: Any] = ["page": page]
        // 추가 parameter들 (고정값들)
        parameters["size"] = 25
        
        Network.request(url: NetworkURL.Server.lidars, method: .get, parameters: parameters) { result in
            switch result.status {
            case .SUCCESS:
                guard let data = result.data else {
                    completion(.failure(.empty))
                    return
                }
                
                guard let dto = try? JSONDecoder().decode(LidarInfoDTO.self, from: data) else {
                    completion(.failure(.decode))
                    return
                }
                
                completion(.success(dto))
                
            case .ERROR(let statusCode):
                completion(.failure(.server(statusCode)))
            }
        }
    }
    
    func getLidarDetailInfo(collectId: String, completion: @escaping (Result<LidarDetailInfoDTO, FetchError>) -> Void) {
        Network.request(url: NetworkURL.Server.lidars + "/\(collectId)/detail", method: .get) { result in
            switch result.status {
            case .SUCCESS:
                guard let data = result.data else {
                    completion(.failure(.empty))
                    return
                }
                
                guard let dto = try? JSONDecoder().decode(LidarDetailInfoDTO.self, from: data) else {
                    completion(.failure(.decode))
                    return
                }
                
                completion(.success(dto))
                
            case .ERROR(let statusCode):
                completion(.failure(.server(statusCode)))
            }
        }
    }
    
    func deleteLidar(collectId: String, completion: @escaping (Result<Bool, FetchError>) -> Void) {
        Network.request(url: NetworkURL.Server.deleteLidar(collectId: collectId), method: .delete) { result in
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
