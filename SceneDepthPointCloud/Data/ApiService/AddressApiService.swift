//
//  AddressApiService.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Address 수신받기 위한 Network Service 담당
struct AddressApiService {
    /// KakaoAPI 를 통해 AddressFromGpsDTO 를 반환하는 함수
    func getCoordToAddress(x: Double, y: Double, completion: @escaping (Result<AddressFromGpsDTO, FetchError>) -> Void) {
        let parameters = ["x": x, "y": y]
        Network.request(url: NetworkURL.Kakao.coordToAddress, method: .get, parameters: parameters) { result in
            switch result.status {
            case .SUCCESS:
                guard let data = result.data else {
                    completion(.failure(FetchError.empty))
                    return
                }
                
                guard let dto = try? JSONDecoder().decode(AddressFromGpsDTO.self, from: data) else {
                    completion(.failure(FetchError.decode))
                    return
                }
                
                completion(.success(dto))
                
            case .ERROR(let statusCode):
                completion(.failure(FetchError.server(statusCode)))
            }
        }
    }
}
