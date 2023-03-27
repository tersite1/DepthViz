//
//  NetworkStatus.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

enum NetworkStatus: String {
    case SUCCESS // 200~204
    case ERROR
    
    static func status(_ statusCode: Int) -> NetworkStatus {
        switch statusCode {
        case 200: return .SUCCESS
        default: return .ERROR
        }
    }
}
