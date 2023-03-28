//
//  NetworkURL.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct NetworkURL {
    static let baseURL: String = Bundle.main.infoDictionary!["BASE_URL"] as! String
    static let domain = baseURL + "/LidarDemo"
    static let uploadPly: String = domain + "/v1/collect/building"
}
