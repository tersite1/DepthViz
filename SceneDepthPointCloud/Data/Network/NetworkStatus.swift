//
//  NetworkStatus.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

enum NetworkStatus {
    case SUCCESS
    case ERROR(Int)
    
    static func status(_ statusCode: Int) -> NetworkStatus {
        switch statusCode {
        case (200...299):
            return .SUCCESS
        default:
            return .ERROR(statusCode)
        }
    }
}
