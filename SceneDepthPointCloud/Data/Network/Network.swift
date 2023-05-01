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
    static func request(url: String, method: HTTPMethod, parameters: [String: Any]? = nil, completion: @escaping (NetworkResult) -> Void) {
        AF.request(url, method: method, parameters: parameters, interceptor: NetworkInterceptor())
            .validate()
            .response { response in
                completion(Network.configurationNetworkResult(response))
            }
            .resume()
    }
    // MARK: Content-Type=multipart/form-data 형식으로 upload하는 함수
    static func uploadData(url: String, address: String, location: String, file: LiDARData, completion: @escaping (NetworkResult) -> Void) {
        // multipart/form-data 인코딩
        let multipartFormData = MultipartFormData()
        multipartFormData.append(address.data(using: .utf8)!, withName: "address", mimeType: "application/json")
        multipartFormData.append(location.data(using: .utf8)!, withName: "location", mimeType: "application/json")
        multipartFormData.append(file.lidarData, withName: "file", fileName: file.lidarFileName, mimeType: "application/octet-stream")
        
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
        print("Network Request: \(String(describing: response.request))")
        
        guard let statusCode = response.response?.statusCode else {
            print("Network Fail: No Status Code: \(String(describing: response.error?.localizedDescription))")
            return NetworkResult(data: nil, status: NetworkStatus.status(-1))
        }
        
        let status = NetworkStatus.status(statusCode)
        
        guard let data = response.data else {
            print("Network Warning: No Data")
            return NetworkResult(data: nil, status: status)
        }
        
        // check 용 출력
        print("statusCode: \(statusCode)")
        print("Data: \(String(data: data, encoding: .utf8)!)")
        
        return NetworkResult(data: data, status: status)
    }
}
