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
    func fetchAddress(from location: LocationData, completion: @escaping (String?) -> Void) {
        let endpoint = AddressApiService()
        
        let x = Double(location.longitude)
        let y = Double(location.altitude)
        
        endpoint.fetchAddress(x: x, y: y, completion: completion)
    }
}
