//
//  FetchError.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Network 를 통한 정보수신 중 에러값
enum FetchError: Error {
    case client
    case server(Int)
    case empty
    case decode
    case basic
    
    var message: String {
        switch self {
        case .client:
            return "Client Error"
        case .server(let code):
            return "Server Error: \(code)"
        case .empty:
            return "Server Error: No Data"
        case .decode:
            return "Decode Error"
        case .basic:
            return "Error"
        }
    }
    
    /// statusCode 값을 통해 FetchError 를 반환하는 함수
    static func error(type: FetchError, _ statusCode: Int = 0) -> FetchError {
        switch type {
        case .server(_):
            return .server(statusCode)
        default:
            return type
        }
    }
}
