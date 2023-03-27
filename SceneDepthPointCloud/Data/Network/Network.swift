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
        AF.request(url, method: method)
            .validate()
            .response { response in
                completion(Network.configurationNetworkResult(response))
            }
            .resume()
    }
    
    static func uploadData(url: String, fileName: String, fileData: Data, completion: @escaping (NetworkResult) -> Void) {
        // multipart/form-data 인코딩
        let multipartFormData = MultipartFormData()
        multipartFormData.append(fileData, withName: fileName, mimeType: "application/octet-stream")
        
        // 파일 업로드
        AF.upload(multipartFormData: multipartFormData, to: url)
        .validate()
        .response { response in
            completion(Network.configurationNetworkResult(response))
        }
        .resume()
    }
    
    /// statusCode 값과 Data 값으로 NetworkResult 반환
    static func configurationNetworkResult(_ response: AFDataResponse<Data?>) -> NetworkResult {
        print("Network Request: \(String(describing: response.request?.url))")
        
        guard let statusCode = response.response?.statusCode else {
            print("Network Fail: No Status Code: \(String(describing: response.error?.localizedDescription))")
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
