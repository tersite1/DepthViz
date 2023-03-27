//
//  Network.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import Alamofire

struct Network {
    /// Network 실제 통신 -> configurationNetworkResult 함수를 통해 NetworkResult 반환
    static func request(url: String, method: HTTPMethod, completion: @escaping (NetworkResult) -> Void) {
        Session.default.request(url, method: method)
            .validate()
            .response { response in
                completion(Network.configurationNetworkResult(response))
            }
            .resume()
    }
    
    /// statusCode 값과 Data 값으로 NetworkResult 반환
    static func configurationNetworkResult(_ response: AFDataResponse<Data?>) -> NetworkResult {
        guard let statusCode = response.response?.statusCode else {
            print("Network Fail: No Status Code: \(response.error?.localizedDescription ?? "nil")")
            return NetworkResult(data: nil, status: .ERROR)
        }
        
        let status = NetworkStatus.status(statusCode)
        
        guard let data = response.data else {
            print("Network Warning: No Data")
            return NetworkResult(data: nil, status: status)
        }
        
        return NetworkResult(data: data, status: status)
    }
}
