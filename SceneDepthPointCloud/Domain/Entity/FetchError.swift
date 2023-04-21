//
//  FetchError.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

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
    
    static func error(type: FetchError, _ statusCode: Int = 0) -> FetchError {
        switch type {
        case .server(_):
            return .server(statusCode)
        default:
            return type
        }
    }
}
