//
//  AddressRepositoryInterface.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Data Layer를 간접접근하기 위한 인터페이스
protocol AddressRepositoryInterface: AnyObject {
    /// LocationData로부터 도로명주소를 받는 함수
    func fetchAddress(from location: LocationData, completion: @escaping (Result<String, FetchError>) -> Void)
}
