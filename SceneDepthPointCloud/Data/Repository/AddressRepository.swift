//
//  AddressRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// 주소 정보 Repository
final class AddressRepository: AddressRepositoryInterface {
    /// AddressApiService로부터 AddressFromGpsDTO를 받아 String 값만 반환하는 함수
    func fetchAddress(from location: LocationData, completion: @escaping (Result<String, FetchError>) -> Void) {
        let endpoint = KakaoApiService()
        
        let x = Double(location.longitude)
        let y = Double(location.latitude)
        
        endpoint.getCoordToAddress(x: x, y: y) { result in
            switch result {
            case .success(let addressDTO):
                guard let document = addressDTO.documents.last else {
                    completion(.failure(.basic))
                    return
                }
                
                let address = document.roadAddress.addressName
                completion(.success(address))
                
            case .failure(let error):
                completion(.failure(error))
            }
            
        }
    }
}
